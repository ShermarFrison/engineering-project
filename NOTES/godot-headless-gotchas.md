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
