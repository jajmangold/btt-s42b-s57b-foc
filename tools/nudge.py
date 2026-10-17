#!/usr/bin/env python3
"""Move the axis a bounded distance over SWD. No UART needed.

    python tools/nudge.py <mm>          e.g.  1.0   or  -1.0
    python tools/nudge.py --status

For positioning a gantry by hand -- creeping up to a sensor to fit a magnet,
say. The firmware CLAMPS the request to NUDGE_MAX_REV and runs it at
NUDGE_mA with a reduced velocity limit, so a mistake here moves the axis a
little, slowly, not a lot, quickly.

THIS MOVES THE MACHINE. On a ballscrew with no working limit sensor, nothing
but the clamp and your attention is stopping it.
"""
import os, re, struct, subprocess, sys

OCD = r"C:\Users\Josh\.platformio\packages\tool-openocd"
NM  = r"C:\Users\Josh\.platformio\packages\toolchain-gccarmnoneeabi\bin\arm-none-eabi-nm.exe"
# Which build to resolve symbols against. There is more than one board now,
# and they do not run the same binary -- symbol addresses differ between
# builds, so pointing this at the wrong one yields plausible nonsense.
#   set S57B_ENV=s57b   (or axis_ballscrew, s42b, ...)
ENV = os.environ.get("S57B_ENV", "axis_ballscrew")
ELF = os.path.join(os.path.dirname(__file__), "..", ".pio", "build", ENV, "firmware.elf")

LEAD_MM   = 5.0      # 1605 ballscrew; must match AXIS_LEAD_MM in config.h
CMD_NUDGE = 16

# struct Telem, in DECLARATION ORDER. Offsets are DERIVED from this, never
# written down by hand: hand-counted offsets have silently misread this struct
# three separate times in this project, each time producing numbers that looked
# plausible and were wrong. Add a field here when you add one in main.cpp.
TELEM = [
    ("magic","I"), ("seq","I"),
    ("raw","H"), ("stat","H"), ("raw_min","H"), ("raw_max","H"),
    ("linkfail","I"),
    ("angle","f"), ("shaft","f"), ("vel","f"),
    ("cmd","I"), ("cmd_done","I"),
    ("theta_e","f"), ("move_counts","i"), ("expect_counts","f"),
    ("duty_amp","H"), ("drive_mA","H"),
    ("probe0","i"), ("probe1","i"), ("probe2","i"), ("probe3","i"),
    ("set_mA","H"), ("_pad","H"),
    ("zero_elec","f"), ("sensor_dir","i"), ("target","f"), ("foc_ready","I"),
    ("can_ok","I"), ("can_btr","I"), ("can_tx","I"), ("can_rx","I"),
    ("can_err","I"), ("can_selftest","I"), ("crc_fails","I"), ("status_fails","I"),
    ("jog_vel","f"), ("jog_stops","I"),
    ("gear_k","f"), ("gear_master","f"), ("gear_err","f"), ("gear_err_max","f"),
    ("gear_on","I"),
    ("step_count","i"), ("step_raw","I"), ("step_on","I"),
    ("home_raw","I"), ("home_trig","I"), ("home_state","I"), ("homed","I"),
    ("home_angle","f"),
    ("sysclk_hz","I"), ("clock_ok","I"),
    ("nudge_rev","f"), ("cal_done","I"), ("cal_saved","I"), ("cal_active","I"), ("nudge_vel","f"), ("safety","I"),
]

def _offsets():
    off, out, acc = 0, {}, "<"
    for name, code in TELEM:
        out[name] = struct.calcsize(acc)
        acc += code
    return out

OFF = _offsets()


def sym(name):
    out = subprocess.run([NM, os.path.abspath(ELF)], capture_output=True, text=True).stdout
    m = re.search(r"^([0-9a-fA-F]+)\s+\w+\s+%s$" % name, out, re.M)
    if not m:
        sys.exit("%s not found in %s -- build -e axis_ballscrew first" % (name, ELF))
    return int(m.group(1), 16)


def ocd(cmds):
    args = [os.path.join(OCD, "bin", "openocd.exe"), "-s",
            os.path.join(OCD, "share", "openocd", "scripts"),
            "-f", "interface/stlink-hla.cfg", "-f", "target/stm32f1x.cfg", "-c", "init"]
    for c in cmds:
        args += ["-c", c]
    args += ["-c", "shutdown"]
    r = subprocess.run(args, capture_output=True, text=True, cwd=OCD)
    return r.stdout + r.stderr


def read_words(base, off, n=1):
    out = ocd(["halt", "mdw 0x%08X %d" % (base + off, n), "resume"])
    m = re.search(r"0x[0-9a-f]+:((?:\s+[0-9a-f]+)+)", out)
    if not m:
        sys.exit("read failed:\n" + out[-600:])
    return [int(x, 16) for x in m.group(1).split()]


def status(base):
    # Reading immediately after a reset returns BSS zeros, because the main
    # loop has not yet populated g_telem. That looks exactly like "sensor
    # triggered, angle zero". Give it a moment rather than believing it.
    ang = struct.unpack("<f", struct.pack("<I", read_words(base, OFF["angle"])[0]))[0]
    ctl = struct.unpack("<f", struct.pack("<I", read_words(base, OFF["shaft"])[0]))[0]
    rdy = read_words(base, OFF["foc_ready"])[0]
    raw, trg = read_words(base, OFF["home_raw"], 2)
    # Two frames, and the difference matters: targets live in the CONTROL
    # frame, which is negated from the encoder when sensor_direction is -1.
    print("  encoder    %+.4f rad  (raw sensor)" % ang)
    print("  control    %+.4f rad  (%+.2f mm) <- targets are in THIS frame"
          % (ctl, ctl * LEAD_MM / 6.283185))
    print("  foc_ready  %d%s" % (rdy, "" if rdy else "   <-- run align (cmd 5) first"))
    print("  sensor     pin=%d  %s" % (raw, "TRIGGERED" if trg else "clear"))
    return rdy


def main():
    base = sym("g_telem")
    if len(sys.argv) < 2 or sys.argv[1] in ("--status", "-s"):
        status(base); return 0

    mm  = float(sys.argv[1])
    rev = mm / LEAD_MM
    if not status(base):
        sys.exit("refusing to move: FOC not aligned")
    print("\n  nudging %+.2f mm (%+.3f rev) ..." % (mm, rev))

    word = struct.unpack("<I", struct.pack("<f", rev))[0]
    out = ocd(["halt",
               "mww 0x%08X 0x%08X" % (base + OFF["nudge_rev"], word),
               "mww 0x%08X 0" % (base + OFF["cmd"]),      # re-arm the dispatcher
               "resume", "halt",
               "mww 0x%08X %d" % (base + OFF["cmd"], CMD_NUDGE),
               "resume"])
    if "Error" in out:
        print(out[-600:]); sys.exit("openocd failed")
    print("  sent. re-run with --status to see where it ended up.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
