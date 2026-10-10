# First full run — verification, 2026-10-10

Branch `feature/robot-hide-and-seek`, Godot 4.7.2. Evidence for Task 19 of the plan. The
feature is **not verified** until the user has watched a live round (Step 6).

Times in the visual checklist are the HUD round time `t` (head start included; seeking starts
at t = 15 s). The round results JSON printed by `main.gd` (copied from
`~/.local/share/godot/app_userdata/Robot Hide and Seek/logs/godot.log`) gives the exact
transition times in the same clock; they are quoted as "log".

## Test suite

`./run_tests.sh` → exit 0, 0 WARNING lines, wall clock 2m06s.

```
Overall Summary: 160 test cases | 0 errors | 0 failures | 0 flaky | 0 skipped | 0 orphans |
```

Re-run after the tuning change below: same line, exit 0, 0 WARNING lines. No test was changed.

## Headless batch

```bash
time ./run_experiment.sh res://experiments/ablation.tres 5 experiments/ablation_smoke.csv --log-transitions
python3 -I experiments/analyze.py experiments/ablation_smoke.csv
```

Exit 0, no ERROR or WARNING lines in either run.

**Before tuning** (code as of commit 203afdf), wall clock 3m24s:

```
all_sensors              n=5    catch_rate=0.000 mean_catch_time=nan dist=86.3
no_vision                n=5    catch_rate=0.000 mean_catch_time=nan dist=95.5
no_lidar                 n=5    catch_rate=0.467 mean_catch_time=53.5 dist=143.3
no_ultrasonic_bumper     n=5    catch_rate=0.000 mean_catch_time=nan dist=73.9
vision_only              n=5    catch_rate=0.333 mean_catch_time=54.4 dist=113.0
```

`all_sensors` was **not** higher than `vision_only` or `no_vision`: it caught nothing. The
transitions CSV showed why: 18.6 Patrol→Investigate ("lidar blob") and 16.2
Investigate→Patrol ("nothing found") per round; the seeker kept re-investigating crates.

**After tuning** (final numbers), wall clock 2m57s:

```
all_sensors              n=5    catch_rate=0.933 mean_catch_time=77.9 dist=88.0
no_vision                n=5    catch_rate=0.000 mean_catch_time=nan dist=109.9
no_lidar                 n=5    catch_rate=0.467 mean_catch_time=53.5 dist=143.3
no_ultrasonic_bumper     n=5    catch_rate=0.733 mean_catch_time=72.3 dist=85.0
vision_only              n=5    catch_rate=0.333 mean_catch_time=54.4 dist=113.0
```

| variant | catch rate | mean catch time (s) | seeker distance (m) | seeker collisions | seeker stuck |
|---|---|---|---|---|---|
| all_sensors | 0.933 | 77.9 | 88.0 | 3.8 | 0.4 |
| no_vision | 0.000 | – | 109.9 | 4.0 | 0.2 |
| no_lidar | 0.467 | 53.5 | 143.3 | 18.0 | 4.8 |
| no_ultrasonic_bumper | 0.733 | 72.3 | 85.0 | 2.8 | 2.6 |
| vision_only | 0.333 | 54.4 | 113.0 | 3.8 | 10.6 |

What I see at n = 5 (seeds 0–4):

- `all_sensors` (0.933) is now higher than `vision_only` (0.333) and `no_vision` (0.000).
- The variants without lidar did not change (the tuned knobs are only reached through lidar
  blobs): `no_lidar` and `vision_only` rows are identical before and after.
- `no_vision` catches nothing in either run. Its seeker does reach Chase→Catch (2.6 per round)
  on near blobs, but those blind catches hit crates or nothing. Not investigated here.
- Without lidar the seeker collides far more (18.0 per round for `no_lidar`) and gets stuck more
  (`vision_only` 10.6 stuck events per round): lidar steering matters for motion.
- Catches are mostly all-or-nothing per seed (0 or 3 hiders): static hiders converge on the same
  nook, so finding one finds all three.

## Determinism

```bash
./run_experiment.sh res://experiments/strategies.tres 2 experiments/det_a.csv
./run_experiment.sh res://experiments/strategies.tres 2 experiments/det_b.csv
diff experiments/det_a.csv experiments/det_b.csv && echo IDENTICAL
```

- Before tuning: `IDENTICAL` (md5 8106d48ccff7b5888e76f6c89cce6030 for both files).
- After tuning: `IDENTICAL` (md5 bb2ca1fe667306b584eceb68d5dfc078 for both files).
- The parallel A/B runs used during tuning (5 processes × 2 rounds with `--seed-offset`)
  reproduced the single-process rows for seeds 0–4 byte for byte, and two identical tuned runs
  gave identical CSVs.
- Side observation (editor, not headless): the editor's first round (seed 1) matched the batch
  row for seed 1 exactly; rounds started afterwards with R (seeds 2–5) differed from their batch
  rows. Recorded in `NOTES/godot-headless-gotchas.md` item 6; not investigated.

## Visual run checklist

Method: manual calls to the computer-use-vnc MCP tools (screenshots of the game window every
~2.5–8 s) on the main scene run from the editor (F5, after Project → Reload Current Project),
overlay on (F1). Five rounds were watched: seeds 1, 2, 3, 4, 5 (R restarts with seed + 1).
Results: seed 1 caught 3/3 at seek 84.8 s; seed 2 3/3 at 44.0 s; seed 3 3/3 at 64.9 s;
seed 4 0/3 (timeout); seed 5 2/3 (timeout). Stills cannot show continuous motion; where a
check depends on that, it is marked partial.

Seeker:

- [~] **Wait→Patrol at 15 s** — partial. HUD showed "Seeker: Wait" at t = 11.7 (seed 1),
  13.3 (seed 2), 12.7 (seed 4) and the seeker moving by t = 15.4–16.1, phase flipping to
  "seeking" at seek 0. The "Patrol" label itself was never on screen: Patrol → Investigate
  followed within 0.01–0.56 s in seeds 2–5 (log: Wait→Patrol "head start over" at 15.02 in all
  five rounds).
- [x] **Patrol→Chase on a sighting** — seed 1: Patrol at t = 91.1, Chase at t = 94.1
  (log 91.15 "hider seen"). Investigate→Chase also seen: seed 3 t = 71.3 → 74.2 (log 73.37),
  seed 5 t = 68.2 → 71.0 (log 71.03).
- [x] **Chase→Catch (hider greys out)** — seed 1: HUD went from "caught 0/3" (t = 94.1) to
  Hider1/Hider2 "Caught", "caught 2/3" (t = 97.1); "Seeker: Catch" on screen at t = 99.8
  (seed 1) and t = 75.9 (seed 5). Grey chassis of the three caught hiders next to the red seeker
  seen in a close-up right after seed 2 ended (t = 59.0): `docs/screenshots/catch.png`.
- [~] **Catch→Patrol** — partial. "Seeker: Catch" at t = 75.9 (seed 5) was followed by Chase at
  t = 81.5; the Patrol state between lasted ≤ 0.01 s (log 76.37 "catch done" → 76.38) and was
  never on screen.
- [ ] **Chase→Patrol after losing a hider > 3 s** — not observed (0 times in the 5 editor
  rounds; 1 time in the 5 all-sensors batch rounds).
- [x] **Patrol→Investigate on a lidar blob** — seed 1: Patrol at t = 28.1, Investigate at
  t = 30.7 (log 28.67 "lidar blob").
- [x] **Investigate→Patrol (timeout or nothing found)** — seed 1: Investigate at t = 25.9,
  Patrol at t = 28.1 (log 26.18 "timeout"); Investigate at t = 41.7, Patrol at t = 44.7
  (log 43.57 "nothing found").

Hider:

- [x] **PickSpot→GoToSpot→Hide** — all three hiders in GoToSpot at t = 0.3–1.9; Hider2 in Hide
  at t = 11.7 (seed 1, log 8.43 "arrived"). PickSpot lasts one tick (0.02 s) and is never on
  screen.
- [ ] **Hide→Flee when seen** — not observed as that arrow. Seed 5: Hider1 "Hide" at t = 73.7,
  "Flee" at t = 75.9, but the log shows Hide→Relocate (75.08 "blob approaching") →
  GoToSpot → Flee (75.33 "seeker seen"): the GoToSpot→Flee arrow, not Hide→Flee.
- [x] **Flee→PickSpot after escaping** — seed 5: Hider1 "Flee" at t = 75.9, "GoToSpot" at
  t = 81.5, out of the nook and chased across the north room (log 80.05 "seeker unseen",
  80.07 PickSpot→GoToSpot); it was caught at t = 82.4 (HUD "caught 2/3" at t = 87.3).
- [~] **Hide→Relocate on a lidar blob (hider moves when the seeker approaches unseen)** —
  partial. Hide→GoToSpot via Relocate seen on the HUD: seed 4 Hider2 Hide at t = 12.7 →
  GoToSpot at t = 15.4 (log 14.70 "blob approaching"), Hider0 Hide at t = 19.9 → GoToSpot at
  t = 23.2 (log 21.83). In both the blob was another hider (the seeker was in Wait or in the far
  west room). The seeker-caused case is in the logs (seed 5 75.08, seed 3 79.78, seed 1 99.78)
  but was not seen on screen.

Motion:

- [~] **A robot routes around a crate** — partial, stills only: seed 4 (camera following the
  seeker) the seeker was south of a crate at t = 56.1 and north-west of it at t = 58.8; seed 3
  similar at t = 30.3 → 33.5. The detour path itself was not watched.
- [ ] **Bumper recovery (reverse and turn) after a collision** — not observed. The logs count
  1–5 seeker collisions per round (seeds 1–5: 4, 3, 1, 5, 3), none caught on screen.
- [ ] **No robot spins in place for more than 3 s** — not verified. Stills 2.5–3 s apart cannot
  show this. The seeker did sit in one place for about 3 s several times (seed 2 t = 36.0–39.0,
  vision cone pointing the same way; seed 3 t = 64.3–67.5, cone pointing north then south).

Transition reasons cross-checked against `experiments/ablation_smoke_transitions.csv`
(all_sensors, 5 rounds) and the editor log (5 rounds). Every reason string in the editor log
also occurs in the batch CSV except hider GoToSpot→PickSpot "stuck" (editor 2, batch 0), a
reason the code defines:

| robot | transition | reason | editor (5 rounds) | batch all_sensors (5 rounds) |
|---|---|---|---|---|
| seeker | Wait→Patrol | head start over | 5 | 5 |
| seeker | Patrol→Investigate | lidar blob | 45 | 43 |
| seeker | Investigate→Patrol | nothing found | 39 | 34 |
| seeker | Investigate→Patrol | timeout | 2 | 2 |
| seeker | Patrol→Chase | hider seen | 8 | 8 |
| seeker | Investigate→Chase | hider seen | 3 | 7 |
| seeker | Chase→Catch | in tag range | 11 | 14 |
| seeker | Catch→Patrol | catch done | 8 | 10 |
| seeker | Chase→Patrol | target lost | 0 | 1 |
| hider | PickSpot→GoToSpot | spot chosen | 18 | 16 |
| hider | GoToSpot→Hide | arrived | 41 | 27 |
| hider | GoToSpot→PickSpot | stuck | 2 | 0 |
| hider | Hide→Relocate | blob approaching | 29 | 16 |
| hider | Relocate→GoToSpot | new spot | 29 | 16 |
| hider | GoToSpot→Flee | seeker seen | 1 | 3 |
| hider | Flee→PickSpot | seeker unseen | 1 | 1 |

Screenshots: `docs/screenshots/chase.png` (seed 3, t = 74.2, camera following the seeker,
"Seeker: Chase", heading into the nook with the three hiders), `docs/screenshots/catch.png`
(seed 2, t = 59.0, round just finished 3/3: the red seeker beside three grey caught hiders).

Seen in every watched round: all three static hiders end up in the same north-east nook, and
most Hide→Relocate events (29 in 5 rounds) are hiders reacting to each other's lidar blobs
during the first ~35 s.

## Not yet observed

- Seeker Chase→Patrol after losing a hider for > 3 s.
- Hider Hide→Flee as a direct arrow (only GoToSpot→Flee was seen).
- A hider relocating because the *seeker* approaches unseen (only hider-triggered relocations
  were on screen; the seeker-triggered ones are in the logs only).
- Bumper recovery (reverse and turn) after a collision.
- Confirmation that no robot spins in place for more than 3 s.
- The Patrol label right after Wait (15 s) and right after Catch (both last ≤ 0.6 s).
- A continuous view of a robot detouring around a crate (only before/after stills).

## Tuning changes made

One change, A/B tested with the batch runner (all_sensors only, 10 seeds per arm); details and
the stopping rule in `NOTES/tuning-log.md`.

| knob | before | after |
|---|---|---|
| `SeekerStates.Context.investigate_cooldown` | 10 s | 60 s |
| investigated-memory radius (`interesting_blob`, now `Context.investigated_radius`) | 2.0 m | 3.0 m |

| arm | seeds | caught / hiders | seeker transitions / round | Patrol→Investigate / round | "nothing found" / round |
|---|---|---|---|---|---|
| before | 0–9 | 0.100 | 37.5 | 18.3 | 16.2 |
| after | 0–9 | 0.867 | 24.7 | 9.0 | 6.9 |
| before | 10–19 | 0.100 | 35.3 | 17.2 | 13.8 |
| after | 10–19 | 0.500 | 26.1 | 11.1 | 8.8 |
| cooldown only (radius 2.0) | 0–9 | 0.767 | 25.9 | 10.0 | 7.7 |

Side effect: the blind-catch dud memory (`duds`) shares `investigate_cooldown` as its lifetime,
so the no-vision seeker now ignores a dud spot for 60 s instead of 10 s. `no_vision` caught 0
before and after.

Not tried, because the stopping rule (all-sensors catch rate ≥ 0.4 over ≥ 5 seeds) was met after
this change: a single-ray far-blob filter, `investigate_timeout`, patrol scoring weights,
`HiderStrategy.TOP_K`, `Motion.avoid_dist`. The hider convergence noted above is the obvious
next candidate (`TOP_K`).
</content>
