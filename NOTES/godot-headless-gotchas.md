# Godot 4.7.2 headless gotchas (verified by probes, 2026-10-09)

All verified with the bundled binary in `--headless` mode, Jolt physics.

1. **Differential drive on RigidBody3D needs a frictionless chassis.**
   Setting `state.linear_velocity` / `state.angular_velocity` in
   `_integrate_forces` while the body rests on a floor with default friction
   gives wrong motion (yaw ran at ~2x the commanded rate, speed too low).
   With `physics_material_override.friction = 0.0` (or gravity off / Y lock)
   the body tracks commanded v and ω within ~2 %.
2. **`NavigationRegion3D.bake_navigation_mesh()` parses only the region's own
   children by default.** For a navmesh over the whole arena, bake explicitly:
   `NavigationServer3D.parse_source_geometry_data(nm, src, arena_root)` then
   `bake_from_source_geometry_data(nm, src)`, then assign `region.navigation_mesh`
   (a fresh assignment, not the same object re-set) before adding it to the tree.
3. **Path queries return empty for the first physics frames after a region is
   added.** Map iteration is async by default; with
   `NavigationServer3D.map_set_use_async_iterations(map, false)` paths appear at
   physics frame 3, async at frame 5. Always poll a trivial path query for
   readiness instead of querying right after baking.
4. **gdUnit4 6.2.2 runs headless on 4.7.2.** Install under `addons/gdUnit4`,
   enable plugin in `project.godot`, run `--import` once so the class cache
   exists, then:
   `godot --headless --path . -s res://addons/gdUnit4/bin/GdUnitCmdTool.gd --add res://tests --ignoreHeadlessMode`
   Exit code 0 on success, 100 on failures. Assert API: `assert_float(x).is_equal_approx(e, tol)`,
   `assert_vector(v).is_equal_approx(e, Vector3(tol,tol,tol))`, `assert_array(a).has_size(n)`,
   `assert_bool`, `assert_int`, `assert_dict`. Scene tests: `scene_runner(scene)`,
   `await runner.simulate_frames(n)`, `runner.scene()`.
5. `OS.get_cmdline_user_args()` returns everything after `--` as expected.
6. **A round's outcome depended on where `GameManager.start_round` was called.**
   Before the fix, `start_round` cleared the old round, spawned props and baked the
   navmesh synchronously, so the tick on which the robots spawned (and the physics state
   they spawned into) depended on the caller's frame phase: `call_deferred` from a physics
   tick (BatchRunner), a `_process`/input callback (main.gd, key R), or a direct call from a
   test coroutine. Same seed 7, short round: `call_deferred` then `_process` start gave
   hider distances 11.702 / 12.753 vs 10.955 / 12.366 (3 of 3 processes). This was first
   misread as a "first round of a process differs" effect and worked around with a
   discarded warm-up round.
   Fix: `start_round` only latches the config (phase becomes "setup" at once); the setup
   runs on GameManager's next `_physics_process`, and the first `nav.is_ready()` check
   waits a fixed `SETUP_MIN_TICKS = 10` ticks. Verified 2026-10-10: the call-site scene
   test (`determinism_test.gd`) and back-to-back same-seed rounds without warm-up pass in
   3 of 3 suite runs, and the call-site test with the two starts swapped passes in 3 of 3
   processes; the same test file against the pre-fix build (82ad62c) fails 3 of 3 with
   exactly the distances above, and its back-to-back test fails 3 of 3. The ablation config,
   1 round per variant, gives byte-identical CSVs in 3 fresh processes; the F1-only build
   (dc37c6c, no warm-up) is byte-identical to the pre-fix build with its warm-up round on
   the same seeds: the pre-fix batch rows (after the warm-up) equal the fixed build's rows.
   Later behaviour changes (blind-catch margin, hider spacing, per-robot RNG) change the rows.
7. **`Engine.time_scale` scales the physics step, it does not add ticks.**
   With `time_scale = 8` alone, headless still runs 60 physics ticks per real
   second, each with `delta = 0.13333` (4 probe runs, all 120 ticks in 2 s;
   engine `main.cpp` passes `physics_step * time_scale` to physics, navigation
   and `_physics_process`). A run scaled that way simulates with a coarser step
   than the game and the tests, so it gives different (still deterministic)
   results.
   Fix used by `BatchRunner`: set `Engine.physics_ticks_per_second =
   roundi(60 * scale)` together with `Engine.time_scale = scale`. The step is
   then 1/60 s at any scale (at 8: 480 ticks/s, delta 0.01667). BatchRunner
   checks the step on its first tick and quits with code 2 if it is not 1/60.
   The engine CLI flag `--fixed-fps 60` (before `--`) drops real-time sync, so
   the loop is CPU-bound with a fixed 1/60 s frame (empty scene, 4 probe runs:
   801k-848k ticks in 2 s, delta 0.01667). `run_experiment.sh` passes it.
   Verified: ablation config, 1 round per variant, gives byte-identical CSVs
   for `--fixed-fps 60` at scale 8 (49.5 s and, in a fresh process, 48.5 s),
   `--fixed-fps 60` at scale 1 (49.5 s), and real-time at scale 8 without
   `--fixed-fps` (101.6 s).
