#!/usr/bin/env python3
"""Measure encoder nonlinearity and build a correction table, over SWD.

    python tools/calibrate.py sweep    # rotates one revolution open-loop
    python tools/calibrate.py apply    # compute + upload the table, enable it
    python tools/calibrate.py verify   # re-sweep with correction ON

WHY THIS EXISTS. The TLE5012B is specified around +/-0.5-1 degrees of angle
error, and that error is systematic -- this magnet, this shaft, repeatable. On
a 50-pole-pair stepper every mechanical degree is FIFTY electrical degrees, and
FOC loses torque past ~30 and reverses past 90. The ballscrew axis measured
2.28 deg peak = 114 electrical, and would not run closed loop at all.

THE TABLE IS SPECIFIC TO ONE MAGNET IN ONE POSITION ON ONE MOTOR. Re-run the
sweep after any mechanical change.
"""
import os, re, struct, subprocess, sys, tempfile, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import nudge

N_SWEEP  = 800
BUCKETS  = 256
SHIFT    = 7
FULL     = 32768


def syms():
    out = subprocess.run([nudge.NM, "-S", os.path.abspath(nudge.ELF)],
                         capture_output=True, text=True).stdout
    g = {}
    for name in ("_ZL7cal_raw", "tle_cal_corr", "tle_cal_enable"):
        m = re.search(r"^([0-9a-f]+)\s+([0-9a-f]+)\s+\w+\s+%s$" % name, out, re.M)
        if not m:
            sys.exit("symbol %s missing -- rebuild" % name)
        g[name] = (int(m.group(1), 16), int(m.group(2), 16))
    return g


def read_bytes(addr, nbytes):
    tmp = tempfile.mkdtemp()
    nudge.ocd(["halt", "dump_image %s/b.bin 0x%08X %d" % (tmp.replace("\\","/"), addr, nbytes), "resume"])
    return open(os.path.join(tmp, "b.bin"), "rb").read()


def read_sweep(addr, nbytes):
    tmp = tempfile.mkdtemp()
    nudge.ocd(["halt", "dump_image %s/c.bin 0x%08X %d" % (tmp.replace("\\", "/"), addr, nbytes), "resume"])
    d = open(os.path.join(tmp, "c.bin"), "rb").read()
    return list(struct.unpack("<%dH" % (len(d) // 2), d))


def unwrap(v):
    out = [float(v[0])]
    for k in range(1, len(v)):
        d = v[k] - v[k-1]
        if d >  FULL//2: d -= FULL
        if d < -FULL//2: d += FULL
        out.append(out[-1] + d)
    return out


def residuals(v):
    """Deviation of the unwrapped encoder from the IDEAL line, in counts.

    The slope is NOT fitted. Stepping a 50-pole-pair motor through 50 electrical
    revolutions must return it to exactly where it started, so one sweep spans
    exactly one mechanical revolution: 32768 counts over n steps, sign taken
    from the measured direction. Fitting the slope freely instead lets endpoint
    noise tilt the whole line -- which is what produced a residual of +244 at
    the start and -89 at the end of the same revolution, a 333-count step across
    a seam that is physically continuous.
    """
    un = unwrap(v); n = len(un)
    sign = 1.0 if un[-1] >= un[0] else -1.0
    m = sign * FULL / float(n)              # fixed by geometry, not fitted
    b = sum(un[i] - m*i for i in range(n)) / n   # offset only
    return [un[i] - (m*i + b) for i in range(n)], m, b


def harmonics(res, orders=10):
    """Keep only the low-order content of the residual.

    Encoder angle error is low-order: a handful of cycles per MECHANICAL
    revolution, dominated by magnet eccentricity and tilt. Detent torque is
    not -- it repeats once per full step, 200x per revolution, every 164
    counts. The sweep measures both, because an open-loop park settles where
    detent allows rather than where the field points.

    A 256-bucket table spans 128 counts per bucket and cannot represent a
    164-count period at all, so feeding it the raw residual aliases the detent
    component into nonsense. Worse, detent is not an encoder error and must not
    be corrected: doing so fights the motor's own magnetic preference.

    Fitting a few harmonics keeps the part that is genuinely encoder error and
    discards the rest.
    """
    import math
    n = len(res)
    out = [0.0]*n
    out = [sum(res)/n]*n
    for h in range(1, orders+1):
        c = sum(res[i]*math.cos(2*math.pi*h*i/n) for i in range(n)) * 2.0/n
        sN = sum(res[i]*math.sin(2*math.pi*h*i/n) for i in range(n)) * 2.0/n
        for i in range(n):
            out[i] += c*math.cos(2*math.pi*h*i/n) + sN*math.sin(2*math.pi*h*i/n)
    return out


def report(res, label):
    pk = max(res, key=abs); n = len(res)
    rms = (sum(r*r for r in res)/n) ** 0.5
    print("  %-22s peak %+7.1f counts = %5.3f mech = %6.1f ELEC deg" %
          (label, pk, abs(pk)*360/FULL, abs(pk)*360/FULL*50))
    print("  %-22s rms  %7.1f counts = %5.3f mech = %6.1f elec deg" %
          ("", rms, rms*360/FULL, rms*360/FULL*50))
    return abs(pk)*360/FULL*50


def do_sweep(S, enable):
    base = nudge.sym("g_telem")
    nudge.ocd(["halt", "mwb 0x%08X %d" % (S["tle_cal_enable"][0], 1 if enable else 0),
               "mww 0x%08X 800" % (base + nudge.OFF["set_mA"]), "resume"])
    C = base + nudge.OFF["cmd"]
    nudge.ocd(["halt", "mww 0x%08X 0" % C, "resume", "halt", "mww 0x%08X 19" % C, "resume"])
    time.sleep(N_SWEEP * 2 * 0.06 + 10)
    both = read_sweep(S["_ZL7cal_raw"][0], N_SWEEP * 4)
    fwd, rev_raw = both[:N_SWEEP], both[N_SWEEP:]
    rev = rev_raw[::-1]          # reverse pass was recorded in descending k
    # circular mean, because readings wrap at 32768
    out = []
    for a, b in zip(fwd, rev):
        d = b - a
        if d >  FULL//2: d -= FULL
        if d < -FULL//2: d += FULL
        out.append(int(round(a + d/2.0)) % FULL)
    hyst = [abs(((b-a+FULL//2) % FULL) - FULL//2) for a, b in zip(fwd, rev)]
    print("  hysteresis between passes: mean %.1f counts (%.1f elec deg), max %d"
          % (sum(hyst)/len(hyst), sum(hyst)/len(hyst)*360/FULL*50, max(hyst)))
    return out


def main():
    what = sys.argv[1] if len(sys.argv) > 1 else "sweep"
    S = syms()

    if what in ("sweep", "apply"):
        print("  sweeping with correction OFF (one open-loop revolution)...")
        v = do_sweep(S, enable=False)
        res, m, b = residuals(v)
        report(res, "UNCORRECTED")
        open("cal_sweep.txt", "w").write("\n".join(map(str, v)))
        if what == "sweep":
            return 0

        # Keep only low-order content: that is the encoder error. What is left
        # is detent and noise, which the table cannot represent and must not
        # try to correct.
        smooth = harmonics(res, orders=10)
        resid_after = [res[i]-smooth[i] for i in range(len(res))]
        report(smooth, "ENCODER (low-order)")
        report(resid_after, "DISCARDED (detent+noise)")
        pts = sorted(((v[k] & 0x7FFF, -smooth[k]) for k in range(len(v))), key=lambda t: t[0])
        table = []
        for bkt in range(BUCKETS):
            centre = bkt * (FULL // BUCKETS) + (FULL // BUCKETS) // 2
            lo = max((p for p in pts if p[0] <= centre), key=lambda t: t[0], default=pts[-1])
            hi = min((p for p in pts if p[0] >= centre), key=lambda t: t[0], default=pts[0])
            if hi[0] == lo[0]:
                c = lo[1]
            else:
                f = (centre - lo[0]) / float(hi[0] - lo[0])
                c = lo[1] + f * (hi[1] - lo[1])
            table.append(int(round(c)))
        print("  table: %d buckets, correction range %+d .. %+d counts"
              % (BUCKETS, min(table), max(table)))

        addr = S["tle_cal_corr"][0]
        blob = b"".join(struct.pack("<h", t) for t in table)
        tmp = tempfile.mkdtemp(); fp = os.path.join(tmp, "t.bin")
        open(fp, "wb").write(blob)
        nudge.ocd(["halt", "load_image %s 0x%08X bin" % (fp.replace("\\", "/"), addr),
                   "mwb 0x%08X 1" % S["tle_cal_enable"][0], "resume"])
        print("  uploaded and ENABLED.")
        return 0

    if what == "refine":
        # Sweep WITH the table live, then fold the remaining error into it.
        # Converges because each pass only has to cancel what the last one left.
        print("  sweeping with correction ON, to measure what is left...")
        v = do_sweep(S, enable=True)
        res, m, b = residuals(v)
        before = report(res, "BEFORE refine")
        cur = list(struct.unpack("<%dh" % BUCKETS,
                   bytes(bytearray(read_bytes(S["tle_cal_corr"][0], BUCKETS*2)))))
        pts = sorted(((v[k] & 0x7FFF, -res[k]) for k in range(len(v))), key=lambda t: t[0])
        for bkt in range(BUCKETS):
            centre = bkt * (FULL // BUCKETS) + (FULL // BUCKETS) // 2
            lo = max((p for p in pts if p[0] <= centre), key=lambda t: t[0], default=pts[-1])
            hi = min((p for p in pts if p[0] >= centre), key=lambda t: t[0], default=pts[0])
            c = lo[1] if hi[0] == lo[0] else lo[1] + (centre-lo[0])/float(hi[0]-lo[0])*(hi[1]-lo[1])
            cur[bkt] = int(round(cur[bkt] + c))
        blob = b"".join(struct.pack("<h", t) for t in cur)
        tmp = tempfile.mkdtemp(); fp = os.path.join(tmp, "t.bin")
        open(fp, "wb").write(blob)
        nudge.ocd(["halt", "load_image %s 0x%08X bin" % (fp.replace("\\","/"), S["tle_cal_corr"][0]),
                   "mwb 0x%08X 1" % S["tle_cal_enable"][0], "resume"])
        print("  refined table uploaded, range %+d .. %+d" % (min(cur), max(cur)))
        return 0

    if what == "verify":
        print("  re-sweeping with correction ON...")
        v = do_sweep(S, enable=True)
        open("cal_verify.txt", "w").write("\n".join(map(str, v)))
        nudge.ocd(["halt", "mwb 0x%08X 1" % S["tle_cal_enable"][0], "resume"])
        res, m, b = residuals(v)
        left = report(res, "CORRECTED")
        print()
        print("  FOC needs roughly +/-20-30 electrical degrees.")
        print("  VERDICT:", "within range" if left < 30 else "still too large")
        return 0
    sys.exit(__doc__)


if __name__ == "__main__":
    sys.exit(main())
