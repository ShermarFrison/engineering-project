# Tuning log

One line per tuning candidate: knob, before -> after, seeds, measured effect, verdict.
Oracle: `run_experiment.sh` batch rows (`--fixed-fps 60`, 1/60 s step, warm-up round
discarded), `all_sensors` variant alone, 10 seeds per arm, run as 5 parallel processes
of 2 rounds each with `--seed-offset` (rows are byte-identical to a single-process run:
checked for seeds 0-4 against the 5-round ablation CSV). "caught" = hiders caught / hiders.

- 2026-10-10, Task 19 — **accepted**: `SeekerStates.Context.investigate_cooldown` 10 -> 60 s and
  investigated-memory radius 2.0 -> 3.0 m (the literal in `interesting_blob` became
  `Context.investigated_radius`). Seeds 0-9: caught 0.100 -> 0.867, seeker transitions/round
  37.5 -> 24.7, Patrol->Investigate "lidar blob" 18.3 -> 9.0, "nothing found" 16.2 -> 6.9,
  seeker collisions 5.3 -> 3.1, seeker stuck 0.0 -> 0.3. Replication on seeds 10-19:
  caught 0.100 -> 0.500, transitions 35.3 -> 26.1. Cause: crates are static, so after 10 s the
  seeker re-investigated the same crates and spent most of the round on them.
  Note: `duds` (blind catches that caught nobody) also use `investigate_cooldown` as TTL,
  so the no-vision seeker now ignores a dud for 60 s instead of 10 s.
- 2026-10-10, Task 19 — attribution only (not a separate change): cooldown 10 -> 60 s with the
  radius left at 2.0, seeds 0-9: caught 0.767 (vs 0.867 with both, 0.100 with neither).
  Both parts contribute; the cooldown carries most of the effect.
- 2026-10-10, Task 19 — not tried (stopping rule met after one accepted change: all-sensors
  caught >= 0.4 over 20 seeds): single-ray far-blob filter, `investigate_timeout`,
  `pick_patrol_target` weights / `local_mark_radius`, `HiderStrategy.TOP_K` 3 -> 5/6,
  `Motion.avoid_dist` 1.5 -> 2.0. Open issue they would address: catches are mostly all-or-nothing
  per seed (0 or 3 hiders in 19 of 20 seeds) because static hiders still converge on the same nook.
- 2026-10-10, final-review fix wave (F5a) — **applied by ruling, not A/B tested on its own**:
  `HiderStrategy.TOP_K` 3 -> 6 (more candidate spots per pick). Present in both arms below.
- 2026-10-10, final-review fix wave (F5b) — **accepted**: hider spacing, `choose_spot` skips spots
  within `HiderStates.Context.hider_spacing` = 3.0 m of a hider in `bb.last_known` (off arm:
  `hider_spacing = 0.0`). Oracle changed for this A/B: build after the F1 fix (no warm-up round
  any more), all sensors, static and dynamic hiders, 10 parallel processes x 2 seeds.
  Fraction of rounds with 0-or-all hiders caught, off -> on:
  seeds 0-19 static 0.70 -> 0.50 (14 -> 10 of 20), dynamic 0.65 -> 0.55 (13 -> 11);
  replication seeds 20-39 static 0.55 -> 0.40 (11 -> 8), dynamic 0.55 -> 0.55 (11 -> 11).
  Pooled 49/80 -> 40/80 (0.61 -> 0.50). Caught/hiders barely moved: seeds 0-19 static
  0.783 -> 0.733, dynamic 0.817 -> 0.800; seeds 20-39 static 0.567 -> 0.567, dynamic
  0.800 -> 0.817. Verdict per the ruling (accept if the 0-or-all fraction falls): accepted.
  The effect is modest (falls in 3 of 4 cells, equal in 1) and half the rounds are still
  0-or-all, so per-hider catch rates remain clustered by round; analyze.py now reports
  hiders caught per round with a CI over rounds for that reason.
