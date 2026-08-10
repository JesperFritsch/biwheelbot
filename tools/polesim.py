#!/usr/bin/env python3
"""Check that the pole tables in src/sensor.cpp can actually be locked onto.

Mirrors pole_try_lock() and sweeps every one of the 44 possible boot positions,
which is the thing that cannot be tested on the bench: the shaft stops wherever
it stops, so a run only ever exercises one of them. The original two-stage
matcher passed on hardware while being unrecoverable for 16 of A's 44 starts.

Run after regenerating either table. Expect 44/44 and no wrong locks.

    python3 tools/polesim.py [--noise]
"""

import math
import random
import re
import sys
from pathlib import Path

N = 44
MIN_QUALITY = 0.10   # POLE_MIN_QUALITY in sensor.cpp
MIN_BEST = 0.50


def load_tables(src):
    """Pull POLE_FULL_A / POLE_FULL_B out of the firmware source."""
    text = Path(src).read_text()
    out = {}
    for name in ("A", "B"):
        m = re.search(rf"POLE_FULL_{name}\[POLE_BINS\]\s*=\s*\{{(.*?)\}};",
                      text, re.S)
        if not m:
            sys.exit(f"could not find POLE_FULL_{name} in {src}")
        vals = [float(v) for v in re.findall(r"([0-9.]+)f", m.group(1))]
        if len(vals) != N:
            sys.exit(f"POLE_FULL_{name} has {len(vals)} entries, expected {N}")
        out[name] = vals
    return out


def derive(full):
    """pole_derive(): per-transition means, and the residual around them."""
    corr, mean = [0.0] * N, [0.0] * 4
    for t in range(4):
        idx = range(t, N, 4)
        mean[t] = sum(full[k] for k in idx) / (N // 4)
        for k in idx:
            corr[k] = full[k] / mean[t]
    return corr, mean


def correlate(a, b, shift):
    """pole_correlate(): Pearson correlation of a against b rotated."""
    ma, mb = sum(a) / N, sum(b) / N
    cov = va = vb = 0.0
    for k in range(N):
        da, db = a[k] - ma, b[(k + shift) % N] - mb
        cov += da * db
        va += da * da
        vb += db * db
    return cov / math.sqrt(va * vb) if va > 0 and vb > 0 else -2.0


def try_lock(f, corr, mtab):
    """pole_try_lock() on an already-normalised live pattern."""
    fres, mlive = [0.0] * N, [0.0] * 4
    for t in range(4):
        idx = range(t, N, 4)
        mlive[t] = sum(f[k] for k in idx) / (N // 4)
        for k in idx:
            fres[k] = f[k] / mlive[t]

    # Stage 1: alignment mod 4. A residue class spans all 11 poles, so rotating
    # the pattern only permutes these four means -- the pole error cancels.
    base, base_err = 0, float("inf")
    for s in range(4):
        e = sum((mlive[t] - mtab[(t + s) % 4]) ** 2 for t in range(4))
        if e < base_err:
            base_err, base = e, s

    # Stage 2: which pole, from the residual.
    best, second, best_s = -2.0, -2.0, base
    for i in range(N // 4):
        s = base + 4 * i
        r = correlate(fres, corr, s)
        if r > best:
            second, best, best_s = best, r, s
        elif r > second:
            second = r

    locked = best >= MIN_BEST and best - second >= MIN_QUALITY
    return locked, best_s, best, second


def rotated(full, S, sigma=0.0, rng=None):
    f = [full[(k + S) % N] * (1 + rng.gauss(0, sigma) if sigma else 1)
         for k in range(N)]
    total = sum(f)
    return [N * x / total for x in f]


def match_dumps(tables, path):
    """Score real 'POLE A,...' dumps against the tables, every shift shown.

    Answers the question a lock/no-lock bit cannot: is the live pattern a
    rotation of the table at all, and is it the same from window to window?
    """
    lines = [l for l in Path(path).read_text().splitlines() if l.startswith("POLE ")]
    if not lines:
        sys.exit(f"no 'POLE A,...' lines found in {path}")

    prev = {}
    for line in lines:
        head, *vals = line.split(",")
        name = head.split()[1]
        f = [float(v) for v in vals]
        if len(f) != N:
            print(f"  {name}: {len(f)} values, expected {N} -- truncated line?")
            continue
        full = tables[name]
        corr, mtab = derive(full)

        mlive = [sum(f[k] for k in range(t, N, 4)) / (N // 4) for t in range(4)]
        fres = [f[k] / mlive[k % 4] for k in range(N)]
        cres = [full[k] / mtab[k % 4] for k in range(N)]

        # Full-pattern correlation over all 44 shifts: if the live data is a
        # rotation of the table at all, exactly one shift stands out here.
        scores = sorted(((correlate(f, full, s), s) for s in range(N)), reverse=True)
        rscores = sorted(((correlate(fres, cres, s), s) for s in range(N)), reverse=True)
        amp = math.sqrt(sum((x - 1.0) ** 2 for x in f) / N)
        tamp = math.sqrt(sum((x - 1.0) ** 2 for x in full) / N)

        print(f"  {name}: rms {amp:.4f} (table {tamp:.4f})   "
              f"means {' '.join(f'{m:.3f}' for m in mlive)}"
              f"  (table {' '.join(f'{m:.3f}' for m in mtab)})")
        print(f"      full    best r={scores[0][0]:+.3f} @{scores[0][1]:2d}   "
              f"next {scores[1][0]:+.3f} @{scores[1][1]:2d}")
        print(f"      residual best r={rscores[0][0]:+.3f} @{rscores[0][1]:2d}   "
              f"next {rscores[1][0]:+.3f} @{rscores[1][1]:2d}")
        if name in prev:
            r = max(correlate(f, prev[name], s) for s in range(N))
            print(f"      vs previous window: best r={r:+.3f} "
                  f"({'reproducible' if r > 0.8 else 'NOT reproducible'})")
        prev[name] = f


def main():
    tables = load_tables(Path(__file__).resolve().parent.parent
                         / "src" / "sensor.cpp")

    if "--match" in sys.argv:
        match_dumps(tables, sys.argv[sys.argv.index("--match") + 1])
        return 0

    failed = False

    for name, full in tables.items():
        corr, mtab = derive(full)
        ok = wrong = 0
        margins = []
        for S in range(N):
            locked, shift, best, second = try_lock(rotated(full, S), corr, mtab)
            if locked and shift == S:
                ok += 1
                margins.append(best - second)
            elif locked:
                wrong += 1
        status = "ok" if ok == N and not wrong else "FAIL"
        print(f"encoder {name}: {ok}/{N} recovered, {wrong} wrong "
              f"(margin {min(margins):.3f}..{max(margins):.3f})  [{status}]")
        failed |= status == "FAIL"

        if "--noise" in sys.argv:
            for sigma in (0.10, 0.25, 0.50):
                for n in (20, 40, 100):
                    rng = random.Random(7)
                    right = wrong = 0
                    for _ in range(2000):
                        S = rng.randrange(N)
                        f = rotated(full, S, sigma / math.sqrt(n), rng)
                        locked, shift, _, _ = try_lock(f, corr, mtab)
                        if locked:
                            right += shift == S
                            wrong += shift != S
                    print(f"    jitter {sigma:.0%} over {n:3d} samples/bin: "
                          f"{right:4d} ok, {wrong:2d} wrong, "
                          f"{2000 - right - wrong:4d} refused")

    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
