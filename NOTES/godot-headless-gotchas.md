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
6. **Open question: the first round played in a process differs from later
   same-seed rounds.** `GameManager.start_round` with the same seed three times
   back to back in one scene (head_start 2, round_time 5, 2 hiders, 6 props,
   seed 7): round 1 gives seeker/hider distances 7.399 / 11.849 / 12.644,
   rounds 2 and 3 both give 6.940 / 11.960 / 12.108 (all other columns equal).
   Reproduced in 3 separate processes (and in 4 more by an earlier probe with
   seed 21); round 1 is itself identical across processes. One A/B hypothesis
   test (3 processes each arm): calling `arena.nav.bake(arena.static_root)` once
   and waiting for `nav.is_ready()` before round 1 makes round 1 equal to rounds
   2 and 3; the same idle wait without that pre-bake does not. So the first
   `NavService.bake` into a fresh navigation map is involved (possibly
   `map_set_use_async_iterations(map, false)` only taking effect after the
   first region was added); the exact mechanism is not confirmed.
   Workaround in use: the determinism tests and `BatchRunner` play one
   discarded warm-up round first.
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
