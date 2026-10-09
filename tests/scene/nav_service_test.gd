extends PhysicsTestSuite

const ARENA := preload("res://scenes/arena/arena.tscn")


func test_nav_becomes_ready_within_30_ticks() -> void:
	var r: Array = await NavArenaHelper.ready_arena(self)
	var arena: Arena = r[1]
	assert_bool(arena.nav.is_ready()).is_true()


func test_path_between_seeker_and_every_hider_spawn() -> void:
	var r: Array = await NavArenaHelper.ready_arena(self)
	var arena: Arena = r[1]
	for h: Vector3 in arena.hider_spawn_points():
		var path: PackedVector3Array = arena.nav.find_path(arena.seeker_spawn_point(), h)
		assert_int(path.size()).append_failure_message("no path to %s" % h).is_greater_equal(2)
		assert_float(path[-1].distance_to(h)).is_less(1.0)


func test_wall_rects_and_bounds() -> void:
	var runner := scene_runner(auto_free(ARENA.instantiate()))
	var arena: Arena = runner.scene()
	assert_that(arena.bounds()).is_equal(Rect2(-15, -15, 30, 30))
	var rects: Array[Rect2] = arena.wall_rects()
	assert_int(rects.size()).is_greater_equal(8)  # 4 outer + interior walls
	for rct: Rect2 in rects:
		assert_bool(Rect2(-15.5, -15.5, 31, 31).encloses(rct)).is_true()


func test_hider_spawns_not_visible_from_seeker_spawn() -> void:
	var r: Array = await NavArenaHelper.ready_arena(self)
	var arena: Arena = r[1]
	var space := arena.get_world_3d().direct_space_state
	var eye: Vector3 = arena.seeker_spawn_point() + Vector3(0, 0.3, 0)
	for h: Vector3 in arena.hider_spawn_points():
		var q := PhysicsRayQueryParameters3D.create(eye, h + Vector3(0, 0.3, 0), 1)
		var hit: Dictionary = space.intersect_ray(q)
		assert_bool(hit.is_empty()).append_failure_message("spawn %s is visible from seeker spawn" % h).is_false()
