#!/usr/bin/env python3
"""Poll g_telem out of the S57B's RAM over SWD. No UART, no extra hardware.

    python tools/telem.py [seconds] [interval_ms]

Reads the target WITHOUT halting it, so the sampling loop keeps running.
"""
import io, os, re, struct, subprocess, sys, tempfile

OCD  = r"C:\Users\Josh\.platformio\packages\tool-openocd"
ELF  = os.path.join(os.path.dirname(__file__), "..", ".pio", "build", "s57b", "firmware.elf")
NM   = r"C:\Users\Josh\.platformio\packages\toolchain-gccarmnoneeabi\bin\arm-none-eabi-nm.exe"
FMT  = "<IIHHHHIfffIIfifHHiiiiHHfifIIIIIIIIIfIffffIiII"  # +step_count step_raw step_on
SIZE = struct.calcsize(FMT)
MAGIC = 0x53354237


def telem_addr():
    out = subprocess.run([NM, os.path.abspath(ELF)], capture_output=True, text=True).stdout
    m = re.search(r"^([0-9a-fA-F]+)\s+\w+\s+g_telem$", out, re.M)
    if not m:
        sys.exit("g_telem not found in %s -- rebuild first" % ELF)
    return int(m.group(1), 16)


def capture(seconds=10.0, interval_ms=200):
    addr = telem_addr()
    n = max(1, int(seconds * 1000 / interval_ms))
    tmp = tempfile.mkdtemp(prefix="telem")
    cmds = ["-f", "interface/stlink-hla.cfg", "-f", "target/stm32f1x.cfg", "-c", "init"]
    for i in range(n):
        cmds += ["-c", "dump_image %s/%03d.bin 0x%08X %d" % (tmp.replace("\\", "/"), i, addr, SIZE)]
        cmds += ["-c", "sleep %d" % interval_ms]
    cmds += ["-c", "exit"]

    r = subprocess.run([os.path.join(OCD, "bin", "openocd.exe"), "-s",
                        os.path.join(OCD, "share", "openocd", "scripts")] + cmds,
                       capture_output=True, text=True, cwd=OCD)
    if "Error" in r.stderr and "dumped" not in r.stderr:
        print(r.stderr[-800:]); sys.exit("openocd failed")

    print("g_telem @ 0x%08X   %d samples @ %d ms" % (addr, n, interval_ms))
    print("%5s %10s %7s %9s %9s %9s %6s" % ("#", "seq", "raw", "deg", "shaft", "vel", "bad"))
    rows = []
    for i in range(n):
        p = os.path.join(tmp, "%03d.bin" % i)
        if not os.path.exists(p):
            continue
        d = io.open(p, "rb").read()
        if len(d) != SIZE:
            continue
        (magic, seq, raw, stat, rmin, rmax, bad, ang, shaft, vel,
         cmd, cmd_done, theta_e, moved, expect, duty, mA,
         p0, p1, p2, p3, set_mA, _pad,
         zero_elec, sensor_dir, target, foc_ready,
         can_ok, can_btr, can_tx, can_rx, can_err, can_self,
         crc_f, stat_f, jog_v, jog_stop,
         g_k, g_m, g_e, g_emax, g_on,
         st_cnt, st_raw, st_on) = struct.unpack(FMT, d)
        if magic != MAGIC:
            print("  !! magic 0x%08X -- firmware not running?" % magic); continue
        rows.append((seq, raw, rmin, rmax, bad, cmd, cmd_done, moved, expect, duty, mA,
                     (p0, p1, p2, p3), zero_elec, sensor_dir, target, foc_ready,
                     can_ok, can_btr, can_tx, can_rx, can_err, can_self, crc_f, stat_f,
                     jog_v, jog_stop, g_k, g_m, g_e, g_emax, g_on,
                     st_cnt, st_raw, st_on))
        print("%5d %10d %7d %9.2f %9.3f %9.3f %6d"
              % (i, seq, raw, raw * 360.0 / 32768.0, shaft, vel, bad))

    if rows:
        raws = [r[1] for r in rows]
        print()
        print("raw seen      : %d .. %d  (span %d of 32768)"
              % (min(raws), max(raws), max(raws) - min(raws)))
        print("lifetime span : %d .. %d" % (rows[-1][2], rows[-1][3]))
        print("sample rate   : %.0f Hz" % ((rows[-1][0] - rows[0][0]) /
                                           (len(rows) * interval_ms / 1000.0)))
        print("link fails    : %d   crc fails: %d   status fails: %d"
              % (rows[-1][4], rows[-1][22], rows[-1][23]))
        L2 = rows[-1]
        if L2[33]:
            print("step/dir      : FOLLOWING   %d steps (%.4f rad)   TIM2 CNT %d"
                  % (L2[31], L2[31]*6.283185/3200.0, L2[32]))
        if L2[30] or L2[29]:
            cnt = lambda r: r * 32768.0 / 6.283185
            print("gearing       : k=%.4f  master=%.4f rad  %s"
                  % (L2[26], L2[27], "ENGAGED" if L2[30] else "released"))
            print("  follow err  : now %+.5f rad (%+.1f counts)   WORST %.5f rad (%.1f counts)"
                  % (L2[28], cnt(L2[28]), L2[29], cnt(L2[29])))
        if rows[-1][24] or rows[-1][25]:
            print("jog           : %.3f rad/s   deadman stops: %d"
                  % (rows[-1][24], rows[-1][25]))
        L = rows[-1]
        if L[15]:
            print("FOC           : READY   zero_electric_angle %.4f rad   sensor_dir %+d"
                  % (L[12], L[13]))
            print("target        : %.4f rad" % L[14])
        elif L[5] == 5:
            print("FOC           : initFOC FAILED -- alignment did not complete")
        if L[16]:
            btr = L[17]
            brp = (btr & 0x3FF) + 1
            ts1 = ((btr >> 16) & 0xF) + 1
            ts2 = ((btr >> 20) & 0x7) + 1
            tq  = 1 + ts1 + ts2
            print("CAN           : %s  BTR=0x%08X  brp=%d TS1=%d TS2=%d"
                  % ("LOOPBACK" if btr & (1 << 30) else "NORMAL", btr, brp, ts1, ts2))
            print("  bitrate     : %d bit/s   sample point %.1f%%"
                  % (36000000 // (brp * tq), 100.0 * (1 + ts1) / tq))
            print("  tx %d  rx %d  err(TEC+REC) %d" % (L[18], L[19], L[20]))
            st = {0: "not run", 1: "PASS -- frame returned intact",
                  2: "FAIL -- frame returned CORRUPTED"}.get(L[21], "no frame returned")
            print("  self-test   : %s" % st)
        last = rows[-1]
        if last[6]:
            print()
            print("last command  : %d (done %d)  drive %d mA, duty %d/255"
                  % (last[5], last[6], last[10], last[9]))
            print("counts moved  : %d" % last[7])
            print("expected      : %.1f" % last[8])
            pr = last[11]
            if last[5] == 3:
                print("coil probe (rotor counts): A+ %d   A- %d   B+ %d   B- %d"
                      % pr)
                print("  A+ vs A- : %+6d counts  %s"
                      % (pr[1]-pr[0], "SAME SPOT => coil A polarity not flipping"
                         if abs(pr[1]-pr[0]) < 40 else "moves => coil A drives both ways"))
                print("  B+ vs B- : %+6d counts  %s"
                      % (pr[3]-pr[2], "SAME SPOT => coil B polarity not flipping"
                         if abs(pr[3]-pr[2]) < 40 else "moves => coil B drives both ways"))
                print("  (one electrical rev = 655 counts; 180 deg elec = 328)")
            if last[8]:
                print("ratio         : %.4f  %s" % (last[7]/last[8],
                      "<- 1.0 means wiring and pole count are both right"))


def send_word(addr, val, name="word"):
    r = subprocess.run([os.path.join(OCD, "bin", "openocd.exe"), "-s",
                        os.path.join(OCD, "share", "openocd", "scripts"),
                        "-f", "interface/stlink-hla.cfg", "-f", "target/stm32f1x.cfg",
                        "-c", "init", "-c", "mwh 0x%08X %d" % (addr, val), "-c", "exit"],
                       capture_output=True, text=True, cwd=OCD)
    print("%s = %d @0x%08X%s" % (name, val, addr, "" if "Error" not in r.stderr else " FAILED"))


def send_cmd(n):
    """Poke g_cmd over SWD. No reflash needed per experiment."""
    addr = telem_addr() + 32          # offset of 'cmd' within Telem
    # clear first: the firmware only acts on a CHANGE of g_cmd, so resending
    # the same command would otherwise be silently ignored.
    r = subprocess.run([os.path.join(OCD, "bin", "openocd.exe"), "-s",
                        os.path.join(OCD, "share", "openocd", "scripts"),
                        "-f", "interface/stlink-hla.cfg", "-f", "target/stm32f1x.cfg",
                        "-c", "init", "-c", "mww 0x%08X 0" % addr, "-c", "sleep 50",
                        "-c", "mww 0x%08X %d" % (addr, n), "-c", "exit"],
                       capture_output=True, text=True, cwd=OCD)
    print("cmd %d -> g_cmd @0x%08X%s" % (n, addr, "" if "Error" not in r.stderr else " FAILED"))


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "cmd":
        if len(sys.argv) > 4:
            send_word(telem_addr() + 72, int(sys.argv[4]), "set_mA")
        send_cmd(int(sys.argv[2]))
        capture(float(sys.argv[3]) if len(sys.argv) > 3 else 6.0, 300)
        sys.exit(0)
    secs = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
    ival = int(sys.argv[2]) if len(sys.argv) > 2 else 200
    capture(secs, ival)
