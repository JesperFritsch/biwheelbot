# Encoder calibration

## The problem

The wheel-speed estimate comes from one number: the time between consecutive
encoder edges. `speed = distance / delta_us`, where the distance is assumed to
be `MM_PER_COUNT` every time.

That assumption is wrong. The edges are not evenly spaced around the shaft, so
some counts cover more arc than others. The reading is then wrong by whatever
that transition's arc actually is, and the error is *not* noise — it repeats at
the same shaft positions every rotation. Averaging does not remove it.

On this robot the raw effect was severe. Encoder B's instantaneous speed
alternated between roughly 1541 and 2544 mm/s while the wheel turned at a
steady 1919 mm/s, because its two channels sit 68.8° apart instead of 90°.
Encoder A, at 89.4°, was close to correct.

Two independent geometry errors turned out to be present.

## 1. Channel phase — `EDGE_SCALE_A` / `EDGE_SCALE_B`

A quadrature cycle is four counts, but if the two channels are not exactly 90°
apart the four edges divide the cycle unequally. With B's 68.8° the arcs run
roughly 19% / 31% / 19% / 31% of a cycle instead of 25% each.

Measured by binning `delta_us` on the decoder index `(prev_state << 2) | state`
and taking each bin's share of the cycle. The index encodes direction as well
as which edge, which matters — comparator hysteresis shifts the switching
angles slightly depending on which way the shaft turns, so forward and reverse
need their own entries. Eight of the sixteen indices are legal transitions; the
rest hold the nominal value.

Shares held within 0.2 percentage points across four duties and both
directions, confirming the error is angular rather than a fixed time offset. A
fixed time offset would shrink in relative terms as the intervals shorten.

**These need no runtime alignment.** The index comes from the pin states, so it
is known absolutely from the first edge after boot.

## 2. Pole spacing — `POLE_FULL_A` / `POLE_FULL_B`

The channel phase is one property shared by all 11 pole pairs, so measuring it
pools every pole into four numbers and averages away how the poles differ from
each other. That residue is what remains: about 3% RMS per edge on A and 6% on
B.

A period scan over candidate periods from 2 to 256 found it repeats every **44
counts**, explaining 51% of A's residual and 67% of B's against a 2.1% chance
baseline. 44 counts is one magnet rotation — 11 pole pairs × 4 quadrature edges
— confirmed by counting increments through one rotation by hand. The 88 and 132
entries that also scored are simply harmonics.

The tables hold each transition's measured arc across one full rotation,
normalised so a flawless encoder reads 1.0 everywhere. `pole_derive()` splits
them at startup into the four per-transition means — which duplicate what
`EDGE_SCALE` already applies — and the residual around those means, which is
the part that is actually new.

What the numbers show:

- **B's magnet is eccentric.** The channel phase *breathes* once per rotation,
  swinging between about 61° and 73° around its 68.8° average as the ring sits
  nearer one sensor then the other. Smooth, single-humped, unmistakable.
- **A's is pole-to-pole scatter**, high spatial frequency, no smooth hump.
- For A the pole error (~2.5%) is *larger* than the channel-phase error (±1.8%)
  that `EDGE_SCALE` corrects, so this table is the only thing that improves A at
  all.

Checks that were run:

- **Removing the wheels changed nothing** (B fwd RMS 24.69% → 23.87% at matched
  rotation period). So the pattern is encoder geometry, not drivetrain speed
  ripple. Wheel imbalance cannot contaminate it anyway: it repeats every 425
  counts, and `gcd(425, 44) = 1`, so it spreads evenly over all 44 bins.
- **The pattern survived a power cycle and a disassembly**, overlaying to ±0.0017
  against an amplitude of 0.139 — about 1% of its own size — once rotated to
  match.
- **Reverse is forward shifted by exactly 2 bins.** Crossing a boundary forward
  sets the count to *k*; crossing the same boundary backward sets it to *k−1*,
  so the arc lands two bins over.

## Why the alignment has to be recovered

The pole table is fixed to the magnet, but the count starts at zero wherever the
shaft happens to be sitting at boot. There is no index pulse, so the offset
between the two is unknown and different every power-up.

The quadrature state pins down `count mod 4`, so only 11 alignments remain — but
11 is not 1, and applying the table at the wrong rotation adds error rather than
removing it.

**This is why the two corrections are kept separate.** Merging them into one
44-entry table would be tidier, but then a bad alignment would scramble the ±25%
channel correction instead of just the ~6% pole term. Split, the worst case of a
failed lock is mild.

## How recovery works

`pole_lock_update()`, called from `loop()`:

1. The ISR keeps a free-running 0–43 index stepped by the count. Its offset from
   the table is exactly the unknown, so the true count is never needed.
2. While unlocked, it accumulates `delta_us` per bin — the same measurement the
   table was built from. Only in one direction, since reverse is offset by two
   bins and mixing the two would blur the pattern.
3. Once every bin holds `POLE_MIN_SAMPLES`, the live pattern is normalised the
   same way the table was, and matched against it in **two stages**.
4. If the peak beats the runner-up by `POLE_MIN_QUALITY`, the table is rotated
   into `pole_live_*` and the ISR starts applying it. Otherwise it keeps
   gathering and tries again — self-pacing, so it works at any speed.

### Why the match needs two stages

The pattern carries two features, and they resolve different things. Using the
wrong one for either job fails silently.

The channel-phase alternation is **identical for every pole**, so it is
invariant under a 4-count shift: it fixes the alignment modulo 4 and nothing
more. The pole residual is unique per pole, and picks among the 11 candidates
that share an alignment.

**Stage 1 — alignment modulo 4, from the four per-transition means.** This works
because a residue class `k mod 4` spans all 11 poles, so its mean does not
depend on which pole the count started at. Rotating the live pattern only
*permutes* the four means, leaving `live_mean[t] == table_mean[(t+S) mod 4]`
exactly. The pole error cancels rather than merely averaging down, so the match
is limited only by measurement noise. Picking the rotation is a least-squares
fit over four numbers.

**Stage 2 — which pole, from the residual.** Divide out the per-transition means
(what `pole_derive()` does to the table) and correlate across the 11 candidates
sharing the alignment. Noiseless, the right shift scores 1.0 against 0.67 for
B's next-best and 0.13 for A's.

#### The mistake worth recording

Stage 1 originally correlated the *whole* 44-bin pattern against the table,
which seems natural and is wrong. The alternation being sought is the **smaller**
feature on encoder A — its channels sit at 89.4°, so the alternation is only
±1.4%, against ±2.4% of pole residual. The residual therefore acts as structured
noise and outvotes the signal.

Simulated over all 44 possible boot positions using the exact tables, that
version chose the wrong base for **16 of A's 44 and 12 of B's**. A wrong base is
unrecoverable: the true shift is not among the 11 candidates stage 2 then
searches, so it can never lock however long it gathers. On the bench this looked
like B locking and A sitting at 2000+ samples per bin forever — and which
encoder failed depended on where the shafts happened to stop at power-down.

Matching the means instead recovers 44/44 on both. `tools/polesim.py` mirrors
the matcher and sweeps every boot position against the tables read straight out
of `sensor.cpp`; run it after regenerating either table, since the bench can
only ever exercise the one start position the shaft happened to stop at.

**Timing.** Roughly 30–50 motor rotations buys the margin. At duty 0.6 the
rotation period is ~16.5 ms, so that is under a second. At balancing speeds —
say 200 mm/s at the wheel, about 8 rotations per second — the same confidence
takes several seconds. Pacing on the correlation rather than a timer means
neither case needs special handling.

**Threshold choice.** With 40 samples per bin, simulation puts both encoders at
2000/2000 correct locks and zero wrong locks at 10% per-sample jitter — several
times the ~1–3% actually left after correction. Degradation is graceful: at 25%
jitter A still takes 1397/2000 with a single wrong lock, the rest refusals. A
confidence guard on stage 1 was tried and dropped; it traded away good locks at
moderate noise without removing the rare wrong ones.

**Missed counts.** An illegal transition means a count was lost, so the mapping
from count to shaft position has slipped and both the lock and the gathered
pattern are invalid. `pole_lock_update()` watches the per-encoder miss counters
and restarts recovery on any change.

**Outlier rejection, and why it is not optional.** The accumulator is a running
sum, so it never forgets: one absurd interval poisons its bin for good. That is
not a rounding-level concern. A single bad bin out of 44 dropped the measured
correlation from **0.98 to 0.25** — the pattern became unmatchable while the
other 43 bins were perfectly good.

The interval is therefore rejected unless it is within 4x of the previous one.
Adjacent transitions differ by at most ~2x (B's widest neighbouring pair), so 4x
is comfortably outside anything geometry can produce, and what it catches is the
wheel having stalled. Smooth speed *changes* need no filtering at all and must
not be filtered: since every bin is visited once per rotation,

```
mean[k] = arc[k] · (1/R) Σ_r 1/ω_r
```

— the speed profile is a constant factor common to all 44 bins and divides out
exactly in the normalisation. A stall is different in kind: it dumps one
enormous interval into whichever single bin was current, and nothing cancels it.

The original instance of this was `last_tick_a/b` starting at zero, so the first
edge after boot timed its interval from power-on — seconds, against a normal few
hundred microseconds. Both are fixed: the timestamps are seeded in
`init_encoders()`, and the ratio test catches the general case.

Because recovery is a background process rather than a startup phase, the robot
can simply start driving and let it converge — or a deliberate spin can be added
at startup to lock it in under a second. Neither requires changing this code.

## Regenerating the tables

Both are per-unit. Swapping or remounting an encoder invalidates them.

1. Drive both motors at a fixed duty (0.6 works: fast enough for a good sample
   rate, slow enough to stay well inside the timing floor).
2. Accumulate `delta_us` in 44 bins indexed by `count mod 44`, per encoder, for
   ~20 s per direction. No detrending is needed — every bin is visited once per
   rotation, so speed drift lands on all 44 equally and cancels in the ratio.
3. Each bin's table value is `44 × mean[k] / sum(all 44 means)`.
4. Ramp between duties rather than stepping, and pass through a full stop before
   reversing, so the gearboxes never take a step change.

`pole_dump()` prints the live accumulator as CSV in exactly this form, and
`tools/polesim.py --match <file>` scores those lines against the current tables
across all 44 shifts — useful both for regeneration and for diagnosing a lock
that will not engage.

Sanity checks: the four per-transition means should reproduce `EDGE_SCALE`, and
two runs in the same power cycle should agree bin for bin. Runs from separate
power cycles will be rotated relative to each other — compare shape, not
indices. Then run `python3 tools/polesim.py`, which must report 44/44 with no
wrong locks for both encoders.
