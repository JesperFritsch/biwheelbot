"""Plot a motor sweep CSV, for building the duty linearisation (ff_duty).

Expects the header:

    tag,motor,duty,v_batt,v_eff,ticks,dt_s,mmps

where tag encodes direction and ramp, e.g. coarse_fwd_up / fine_rev_down.

The point of the sweep is to find, per motor and per direction, the duty at
which the wheel actually starts moving, and whether the response above that is
straight. Both are read off the panels; the breakaway table is also printed.

    python3 tools/plot_sweep.py sweep.csv
    python3 tools/plot_sweep.py sweep.csv --save sweep.png --moving 20

Needs matplotlib (only). Install it in the project venv.
"""

import argparse
import csv
import sys
from collections import defaultdict, namedtuple

try:
    import matplotlib.pyplot as plt
except ImportError:
    sys.exit("matplotlib is not installed -- add it to the project venv "
             "(e.g. `uv pip install matplotlib`) and re-run.")

Row = namedtuple("Row", "tag motor duty v_batt v_eff ticks dt_s mmps "
                        "direction ramp scan")

A_COLOR, B_COLOR = "tab:blue", "tab:orange"
MOTOR_COLOR = {"A": A_COLOR, "B": B_COLOR}


def parse_tag(tag):
    """coarse_fwd_up -> ('coarse', 'fwd', 'up'); tolerant of unknown shapes."""
    parts = tag.lower().split("_")
    scan = parts[0] if parts else tag
    direction = next((p for p in parts if p in ("fwd", "rev", "f", "r")), "fwd")
    ramp = next((p for p in parts if p in ("up", "down", "dn")), "up")
    return scan, ("rev" if direction.startswith("r") else "fwd"), \
        ("down" if ramp.startswith("d") else "up")


def load(path):
    rows = []
    with open(path, newline="") as fh:
        for r in csv.DictReader(fh):
            if not r.get("motor"):
                continue
            scan, direction, ramp = parse_tag(r["tag"])
            rows.append(Row(
                tag=r["tag"], motor=r["motor"].strip().upper(),
                duty=float(r["duty"]), v_batt=float(r["v_batt"]),
                v_eff=float(r["v_eff"]), ticks=int(float(r["ticks"])),
                dt_s=float(r["dt_s"]), mmps=float(r["mmps"]),
                direction=direction, ramp=ramp, scan=scan))
    if not rows:
        sys.exit(f"no data rows in {path}")

    # The log may store magnitudes with the direction only in the tag, or it may
    # store signed values already. Decide once, from the data, and say so.
    rev = [r for r in rows if r.direction == "rev"]
    signed = any(r.duty < 0 or r.mmps < 0 for r in rev)
    if not signed and rev:
        rows = [r._replace(duty=-r.duty, mmps=-r.mmps)
                if r.direction == "rev" else r for r in rows]
        print("reverse rows stored unsigned -- negated for plotting")
    else:
        print("data already signed" if rev else "no reverse rows found")
    return rows


def group(rows):
    """(motor, direction, ramp) -> rows sorted by duty."""
    out = defaultdict(list)
    for r in rows:
        out[(r.motor, r.direction, r.ramp)].append(r)
    for k in out:
        out[k].sort(key=lambda r: r.duty)
    return out


def breakaway(rows, moving):
    """Smallest |duty| at which the wheel was actually turning."""
    turning = [abs(r.duty) for r in rows if abs(r.mmps) >= moving]
    return min(turning) if turning else None


def style(motor, ramp):
    return dict(color=MOTOR_COLOR.get(motor, "tab:green"),
                linestyle="-" if ramp == "up" else "--",
                marker="o", markersize=2.5, linewidth=1.2, alpha=0.85)


def label(motor, direction, ramp):
    return f"{motor} {direction} {ramp}"


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", help="sweep CSV file")
    ap.add_argument("--save", metavar="PNG", help="write the figure to a file")
    ap.add_argument("--moving", type=float, default=20.0,
                    help="mm/s above which the wheel counts as moving (default 20)")
    ap.add_argument("--zoom", type=float, default=0.35,
                    help="x-range of the deadband zoom panel (default 0.35)")
    ap.add_argument("--no-show", action="store_true")
    args = ap.parse_args()

    rows = load(args.csv)
    groups = group(rows)

    # ---- breakaway table -------------------------------------------------
    print(f"\nbreakaway duty (first |duty| with |speed| >= {args.moving:g} mm/s)")
    print(f"  {'motor':<6}{'dir':<6}{'up':>8}{'down':>8}{'stiction':>10}")
    for motor in sorted({r.motor for r in rows}):
        for direction in ("fwd", "rev"):
            up = breakaway(groups.get((motor, direction, "up"), []), args.moving)
            dn = breakaway(groups.get((motor, direction, "down"), []), args.moving)
            if up is None and dn is None:
                continue
            gap = f"{up - dn:+.3f}" if (up is not None and dn is not None) else "-"
            print(f"  {motor:<6}{direction:<6}"
                  f"{'-' if up is None else f'{up:.3f}':>8}"
                  f"{'-' if dn is None else f'{dn:.3f}':>8}{gap:>10}")
    print("  use the 'down' value for ff_duty: while balancing the wheels are\n"
          "  already moving, so the kinetic threshold is the relevant one.\n")

    # ---- figure ----------------------------------------------------------
    fig, ax = plt.subplots(2, 3, figsize=(17, 9))
    fig.suptitle(f"motor sweep -- {args.csv}", fontsize=13)
    (ax_main, ax_zoom, ax_veff), (ax_sag, ax_diff, ax_gain) = ax

    for key in sorted(groups):
        motor, direction, ramp = key
        rs = groups[key]
        st = style(motor, ramp)
        lab = label(*key)

        ax_main.plot([r.duty for r in rs], [r.mmps for r in rs], label=lab, **st)
        ax_zoom.plot([r.duty for r in rs], [r.mmps for r in rs], **st)
        ax_veff.plot([r.v_eff for r in rs], [r.mmps for r in rs], **st)
        ax_sag.plot([abs(r.duty) for r in rs], [r.v_batt for r in rs], **st)

        # incremental gain: how many mm/s per unit duty, locally
        xs, ys = [], []
        for a, b in zip(rs, rs[1:]):
            dd = b.duty - a.duty
            if abs(dd) > 1e-9:
                xs.append(0.5 * (a.duty + b.duty))
                ys.append((b.mmps - a.mmps) / dd)
        ax_gain.plot(xs, ys, **st)

    # A - B mismatch at matched (tag, duty)
    paired = defaultdict(dict)
    for r in rows:
        paired[(r.tag, round(r.duty, 4))][r.motor] = r.mmps
    for ramp, ls in (("up", "-"), ("down", "--")):
        pts = sorted((d, v["A"] - v["B"]) for (tag, d), v in paired.items()
                     if "A" in v and "B" in v and parse_tag(tag)[2] == ramp)
        if pts:
            ax_diff.plot([p[0] for p in pts], [p[1] for p in pts],
                         linestyle=ls, marker="o", markersize=2.5,
                         linewidth=1.2, color="tab:red", label=f"A - B {ramp}")

    # deadband markers on the zoom panel
    for motor in sorted({r.motor for r in rows}):
        for direction, sign in (("fwd", 1), ("rev", -1)):
            bd = breakaway(groups.get((motor, direction, "down"), []), args.moving)
            if bd is not None:
                ax_zoom.axvline(sign * bd, color=MOTOR_COLOR.get(motor, "gray"),
                                linestyle=":", linewidth=1, alpha=0.8)

    ax_main.set(title="speed vs duty (the whole curve)",
                xlabel="duty", ylabel="mm/s")
    ax_main.legend(fontsize=7, ncol=2)

    ax_zoom.set(title=f"deadband zoom -- dotted = 'down' breakaway",
                xlabel="duty", ylabel="mm/s", xlim=(-args.zoom, args.zoom))

    ax_veff.set(title="speed vs effective volts (removes battery sag)",
                xlabel="v_eff = duty x v_batt", ylabel="mm/s")

    ax_sag.set(title="battery sag under load", xlabel="|duty|", ylabel="v_batt")

    ax_diff.set(title="motor mismatch at equal duty", xlabel="duty",
                ylabel="speed A - B  (mm/s)")
    ax_diff.axhline(0, color="k", linewidth=0.8, alpha=0.4)
    if ax_diff.get_legend_handles_labels()[0]:
        ax_diff.legend(fontsize=8)

    ax_gain.set(title="incremental gain -- flat means linear",
                xlabel="duty", ylabel="d(mm/s) / d(duty)")

    for a in ax.flat:
        a.grid(alpha=0.3)
        a.axhline(0, color="k", linewidth=0.8, alpha=0.25)
    for a in (ax_main, ax_zoom, ax_gain):
        a.axvline(0, color="k", linewidth=0.8, alpha=0.25)

    fig.tight_layout(rect=(0, 0, 1, 0.97))

    if args.save:
        fig.savefig(args.save, dpi=140)
        print(f"wrote {args.save}")
    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()
