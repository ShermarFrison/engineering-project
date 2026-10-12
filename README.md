# Robot Hide and Seek

A Godot 4.7.2 simulation of differential-drive robots playing hide and seek in a walled arena
with random crates. One seeker and several hiders each run a finite-state machine (seeker:
Wait, Patrol, Investigate, Chase, Catch; hider: PickSpot, GoToSpot, Hide, Relocate, Flee) on
top of simulated sensors: a 360° lidar, a vision cone with a line-of-sight check, three
ultrasonic cones and a bumper. A headless batch runner plays many seeded rounds and writes
CSV rows for sensor-ablation and hider-strategy experiments.

## Requirements

- Linux x86_64. The Godot binary `Godot_v4.7.2-stable_linux.x86_64` (4.7.2.stable.official,
  Jolt physics) is expected in the project root; it is not tracked by git.
- gdUnit4 6.2.2 is bundled under `addons/gdUnit4` (tests only).
- Python 3 for the analysis script; `matplotlib` is optional (only for the bar chart).

## Run the demo

Open the project in the editor and press F5, or from the command line:

```bash
./Godot_v4.7.2-stable_linux.x86_64 --path .
```

`scenes/main.tscn` plays one round from `experiments/default_round.tres` (seed 1, 3 static
hiders, all sensors, 15 s head start, 120 s seek time). The HUD shows the phase, the timers,
each robot's FSM state and the catches; the round results are printed as JSON when the round
ends.

| key / mouse | action |
|---|---|
| F1 | toggle the debug overlay (lidar rays and vision cone of every robot) |
| R | restart the round with seed + 1 |
| Tab | follow the next robot (seeker first, then the hiders) |
| 0 | free camera |
| right-drag | orbit the camera |
| mouse wheel | zoom |

## Run the tests

```bash
./run_tests.sh
```

Runs the gdUnit4 suite headless (about 2 to 3 minutes). Exit 0 = all passed, 100 = failures,
101 = orphan nodes, 105 = parse error.

## Run experiments

```bash
./run_experiment.sh res://experiments/ablation.tres 30 experiments/ablation.csv
./run_experiment.sh res://experiments/strategies.tres 30 experiments/strategies.csv --log-transitions
```

Arguments: config resource, rounds per variant, output CSV (relative paths are relative to the
project root; absolute paths work too). Optional flags after them:

- `--seed-offset N` shifts the seeds, so a long batch can be split over parallel processes
  (for example 5 processes of 6 rounds with offsets 0, 6, 12, 18, 24). Rows are reproducible
  for an identical command line, but a seed's row can change with the rounds that ran before
  it in the same process (see `NOTES/godot-headless-gotchas.md` item 8). Report which split
  you used, and use the same split when reproducing.
- `--log-transitions` also writes `<out>_transitions.csv` with every FSM transition.
- `--time-scale S` (default 8) speed factor: S simulated seconds per real second when real-time
  synced, or S/60 simulated seconds per frame under `--fixed-fps 60`; the physics step stays 1/60 s.

`run_experiment.sh` passes the engine flag `--fixed-fps 60`, which drops real-time sync so the
run is CPU-bound. Rows do not depend on `--fixed-fps` or `--time-scale`; the runner checks the
1/60 s step on its first tick.

Exit codes of the batch runner:

| code | meaning |
|---|---|
| 0 | every round was recorded |
| 2 | bad arguments or config, output not writable, or physics step not 1/60 s |
| 3 | finished, but at least one round was skipped (setup timed out); the summary lists them, rows already written stay |

### Seeds

Round seeds are `seed_base + --seed-offset + round index` (the same seeds for every variant,
so variants can be compared seed by seed). Tuning used seeds 0–19 (and 20–39 for the hider
spacing A/B); the reported experiments use `seed_base = 1000`, set in both experiment `.tres`
files, so they never reuse a tuning seed.

## Analyse

```bash
python3 -I experiments/analyze.py experiments/ablation.csv
```

Prints per variant: rounds, catch rate, hiders caught per round with a 95 % confidence interval
(normal approximation over rounds), fraction of rounds with at least one catch, fraction of
all-or-nothing rounds, mean and median catch time with the number of timed catches, seeker
distance; then a per-seed table of hiders caught per variant. Writes `<input>_summary.csv`,
`<input>_paired.csv` and, with matplotlib, `<input>_catch_rate.png` next to the input.

## Known limits

- The `stuck` metric counts episodes with less than 0.3 m of straight-line progress toward the
  current goal within 4 s; a robot detouring around a wall can count as stuck.
- `no_vision` measures exploration coverage without vision. The blind seeker can catch (a lidar
  blob within 3 m is a detection), but on the held-out seeds 1000–1029 it never came closer than
  5.2 m to any hider (median closest approach per round 13.8 m), so it caught nothing.
- Results do not depend on where a round is started from (start call site, frame phase); an
  earlier "first round of a process differs" note was a misdiagnosis of that, now fixed. See
  `NOTES/godot-headless-gotchas.md`, item 6.
- A catch is only checked while the seeker requests one (it has detected a hider by vision, or
  a blob when blind); the game manager then validates distance (1.5 m) and line of sight.
- A precomputed `free_sectors` field is not implemented; steering computes clearance per
  candidate heading itself.
- Vision noise (`noise_std` in metres, `bearing_noise_deg`) and lidar/ultrasonic range noise
  exist in `RoundConfig` but the shipped experiments run noise-free.

## Where things are

- `scripts/` game, robot (sensors, FSMs, motion), experiment runner, UI.
- `tests/` gdUnit4 unit and scene tests.
- `experiments/` experiment configs and `analyze.py`.
- `docs/verification/` verification runs with batch tables and the visual checklist.
- `NOTES/` engine findings and the tuning log (`NOTES/INDEX.md` lists them).
