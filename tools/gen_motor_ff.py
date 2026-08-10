#!/usr/bin/env python3
"""Generate include/motor_ff.h from a sweep CSV.

Builds the duty linearisation: a map from requested effort u in [0,1] onto the
duty that actually delivers u of full speed, per motor and per direction.

Only the descending ramps are used. They carry the kinetic dropout, which is the
threshold that matters while balancing (the wheels are already turning), and it
is far more reproducible than static breakaway -- across sweep4 the dropouts
span 0.018 while one motor's breakaway varied by 0.09 between two measurements
minutes apart.

    python3 tools/gen_motor_ff.py sweep4_300us.txt > include/motor_ff.h
    python3 tools/gen_motor_ff.py sweep4_300us.txt --check   # error report only
"""

import argparse
import csv
import sys
from collections import defaultdict

RUNS = [("A", "fwd"), ("A", "rev"), ("B", "fwd"), ("B", "rev")]
MOVING = 20.0      # mm/s above which the wheel counts as turning


def load(path):
    """(motor, direction) -> monotonic [(duty, speed)] from the descents."""
    raw = defaultdict(dict)
    for r in csv.DictReader(open(path)):
        if not r.get("motor"):
            continue
        try:
            tag, m = r["tag"], r["motor"].strip().upper()
            duty, v = abs(float(r["duty"])), abs(float(r["mmps"]))
        except (TypeError, ValueError):
            continue
        if not (tag.endswith("dn") or tag.endswith("down")):
            continue
        direction = "fwd" if "fwd" in tag else "rev"
        if v > MOVING:
            # Both ramps land on the same duties; keep the faster reading, which
            # is the one not caught mid-decay after a step change.
            raw[(m, direction)][round(duty, 4)] = max(
                v, raw[(m, direction)].get(round(duty, 4), 0.0))

    curves = {}
    for key, pts in raw.items():
        seq = sorted(pts.items())
        # Enforce monotonicity so the inverse lookup is well defined. Speed can
        # only rise with duty; a dip is measurement noise, not physics.
        mono, peak = [], 0.0
        for d, v in seq:
            peak = max(peak, v)
            mono.append((d, peak))
        curves[key] = mono
    return curves


def duty_for_speed(curve, target):
    """Inverse of the measured curve, linearly interpolated."""
    if target <= curve[0][1]:
        return curve[0][0]
    for (d0, v0), (d1, v1) in zip(curve, curve[1:]):
        if v0 <= target <= v1:
            return d0 + (d1 - d0) * (target - v0) / (v1 - v0) if v1 > v0 else d0
    return curve[-1][0]


def speed_for_duty(curve, duty):
    if duty <= curve[0][0]:
        return curve[0][1]
    for (d0, v0), (d1, v1) in zip(curve, curve[1:]):
        if d0 <= duty <= d1:
            return v0 + (v1 - v0) * (duty - d0) / (d1 - d0) if d1 > d0 else v0
    return curve[-1][1]


def build(curves, n):
    """Uniform in u, so the runtime lookup is an index and a lerp -- no search."""
    vmax = min(c[-1][1] for c in curves.values())
    tables = {}
    for key, curve in curves.items():
        tab = [curve[0][0]]                      # u = 0 -> the dropout duty
        for i in range(1, n):
            tab.append(duty_for_speed(curve, (i / (n - 1)) * vmax))
        tables[key] = tab
    return vmax, tables


def lookup(tab, u):
    x = u * (len(tab) - 1)
    i = min(int(x), len(tab) - 2)
    return tab[i] + (x - i) * (tab[i + 1] - tab[i])


def report(curves, vmax, tables, n):
    """Accuracy above the floor. Below it nothing can be matched, so scoring
    there measures the motors' physics rather than the table."""
    floors = {k: curves[k][0][1] for k in RUNS}
    u_min = max(floors.values()) / vmax

    print(f"# N={n}  VMAX={vmax:.0f} mm/s", file=sys.stderr)
    print("#   slowest sustainable: "
          + ", ".join(f"{m}{d[0]}={floors[(m, d)]:.0f}" for m, d in RUNS)
          + f" mm/s  ->  matchable only above u={u_min:.2f}", file=sys.stderr)
    print(f"#   {'u':>6}" + "".join(f"{m + ' ' + d:>10}" for m, d in RUNS)
          + f"{'spread':>9}", file=sys.stderr)
    for u in (0.05, 0.1, 0.2, 0.4, 0.6, 0.8, 1.0):
        sp = [speed_for_duty(curves[k], lookup(tables[k], u)) for k in RUNS]
        print(f"#   {u:>6.2f}" + "".join(f"{v:>10.0f}" for v in sp)
              + f"{max(sp) - min(sp):>7.0f}mm/s", file=sys.stderr)

    err = spread = 0.0
    for i in range(int(u_min * 100) + 1, 101):
        u = i / 100
        sp = [speed_for_duty(curves[k], lookup(tables[k], u)) for k in RUNS]
        spread = max(spread, max(sp) - min(sp))
        err = max(err, max(abs(s - u * vmax) for s in sp))
    print(f"# above u={u_min:.2f}: max linearity error {err:.0f} mm/s "
          f"({100 * err / vmax:.1f}% FS), max A/B spread {spread:.0f} mm/s",
          file=sys.stderr)


def emit(path, vmax, tables, n, curves):
    name = {("A", "fwd"): "FF_A_FWD", ("A", "rev"): "FF_A_REV",
            ("B", "fwd"): "FF_B_FWD", ("B", "rev"): "FF_B_REV"}
    v_ref = 10.7
    out = [
        "#pragma once",
        "#include <math.h>",
        "#include <stdbool.h>",
        "",
        "// Duty linearisation, generated by tools/gen_motor_ff.py from",
        f"// {path}. Do not edit by hand -- regenerate.",
        "//",
        "// The motors do nothing below ~0.34 duty and their response above it is",
        "// neither straight nor matched between units, so a raw duty command means",
        "// something different on each motor and direction. These tables map a",
        "// requested effort u in [0,1] onto the duty that actually delivers u of",
        "// full speed, which makes one set of controller gains valid everywhere.",
        "//",
        "// Built from the descending ramps only: they carry the kinetic dropout,",
        "// the threshold that applies while balancing, and it is reproducible in a",
        "// way static breakaway is not.",
        "",
        f"#define FF_N {n}",
        f"static const float FF_VMAX  = {vmax:.1f}f;   // mm/s at u = 1, common to all four",
        f"static const float FF_V_REF = {v_ref:.2f}f;    // pack voltage during the sweep",
        "",
        "// Uniform in u: index = u * (FF_N - 1), so lookup is a lerp, no search.",
    ]
    for key in RUNS:
        vals = tables[key]
        out.append(f"static const float {name[key]}[FF_N] = {{")
        for i in range(0, len(vals), 6):
            out.append("    " + " ".join(f"{v:.4f}f," for v in vals[i:i + 6]))
        out.append("};")
        out.append("")

    out += [
        "static inline const float* ff_table(bool motor_a, bool forward) {",
        "    if (motor_a) return forward ? FF_A_FWD : FF_A_REV;",
        "    else         return forward ? FF_B_FWD : FF_B_REV;",
        "}",
        "",
        "/*",
        " * Requested effort -> the duty that actually produces it.",
        " *",
        " *   u : signed, [-1, 1]. Magnitude is the fraction of full speed wanted;",
        " *       the sign picks the direction and comes back on the result.",
        " *",
        " * u = 0 returns exactly 0. Any u != 0 returns at least the dropout duty,",
        " * so the smallest non-zero command still moves the wheel -- that step is",
        " * physical (static friction), and no mapping removes it. It is the reason",
        " * duty near zero is not usable directly.",
        " */",
        "static inline float ff_duty(float u, bool motor_a) {",
        "    if (u == 0.0f) return 0.0f;",
        "",
        "    bool forward = (u > 0.0f);",
        "    float mag = fabsf(u);",
        "    if (mag > 1.0f) mag = 1.0f;",
        "",
        "    const float* tab = ff_table(motor_a, forward);",
        "    float x = mag * (FF_N - 1);",
        "    int i = (int)x;",
        "    if (i > FF_N - 2) i = FF_N - 2;",
        "    float duty = tab[i] + (x - (float)i) * (tab[i + 1] - tab[i]);",
        "",
        "    if (duty > 1.0f) duty = 1.0f;",
        "    return forward ? duty : -duty;",
        "}",
        "",
        "/*",
        " * Same, with the deadzone corrected for pack voltage.",
        " *",
        " * Friction torque is fixed, so the duty needed to overcome it scales as",
        " * 1/V: the tables were measured near 10.7 V, and on a fresh pack the real",
        " * dropout is lower than they say. Left alone, the smallest non-zero",
        " * command delivers too much torque on a full battery and too little on a",
        " * flat one.",
        " *",
        " * Only the floor moves. Scaling the whole curve by FF_V_REF/v_batt would",
        " * be the honest way to hold u at a fixed speed, but FF_V_REF sits near the",
        " * bottom of this pack's range, so in normal use it would only ever cut --",
        " * capping duty around 0.85 on a charged pack and giving away a sixth of",
        " * the recovery authority. u = 1 means 'everything available' instead, and",
        " * the speed it buys rises with charge. The loop closes around that; it",
        " * cannot close around a deadzone in the wrong place.",
        " */",
        "static inline float ff_duty_compensated(float u, bool motor_a, float v_batt) {",
        "    float duty = ff_duty(u, motor_a);",
        "    if (duty == 0.0f || v_batt <= 1.0f) return duty;   // guard a dead ADC read",
        "",
        "    const float* tab = ff_table(motor_a, duty > 0.0f);",
        "    float floor_ref = tab[0];                       // dropout at FF_V_REF",
        "    float floor_now = floor_ref * (FF_V_REF / v_batt);",
        "    if (floor_now > 0.95f) floor_now = 0.95f;       // absurdly flat pack",
        "",
        "    // Remap [floor_ref, 1] onto [floor_now, 1]: the deadzone shifts, the",
        "    // top of the range stays reachable.",
        "    float mag = fabsf(duty);",
        "    mag = floor_now + (mag - floor_ref) * (1.0f - floor_now) / (1.0f - floor_ref);",
        "    if (mag > 1.0f) mag = 1.0f;",
        "    if (mag < 0.0f) mag = 0.0f;",
        "",
        "    return duty > 0.0f ? mag : -mag;",
        "}",
    ]
    return "\n".join(out) + "\n"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("-n", type=int, default=17, help="table points (default 17)")
    ap.add_argument("--check", action="store_true", help="report only, no header")
    args = ap.parse_args()

    curves = load(args.csv)
    missing = [k for k in RUNS if k not in curves]
    if missing:
        sys.exit(f"missing runs in {args.csv}: {missing}")

    if args.check:
        for n in (5, 9, 13, 17, 21, 25):
            vmax, tables = build(curves, n)
            report(curves, vmax, tables, n)
            print("#", file=sys.stderr)
        return

    vmax, tables = build(curves, args.n)
    report(curves, vmax, tables, args.n)
    sys.stdout.write(emit(args.csv, vmax, tables, args.n, curves))


if __name__ == "__main__":
    main()
