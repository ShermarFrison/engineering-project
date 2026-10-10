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
