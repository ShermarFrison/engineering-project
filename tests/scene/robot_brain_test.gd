extends PhysicsTestSuite

const CRATE := preload("res://scenes/props/crate.tscn")


func _xz(p: Vector3) -> Vector2:
	return Vector2(p.x, p.z)


func test_goto_reaches_goal_around_a_crate() -> void:
	var r: Array = await NavArenaHelper.ready_arena(self)
	var arena: Arena = r[1]
	# Room SW, straight line along z = 8 from (-13, 8) to (-5, 8); a 1.5 x 1 m crate at (-9, 8)
	# sits on the line, 2 m south of the z = 5 divider and north of the nook wall (z >= 10).
	var crate: StaticBody3D = CRATE.instantiate()
	crate.scale = Vector3(1.5, 1.0, 1.0)
	crate.position = Vector3(-9, 0.5, 8)
	arena.props_root.add_child(crate)
	var body := NavArenaHelper.spawn_robot(arena, Vector3(-13, 0, 8), "hider")
	body.rotation.y = PI / 2  # face -X initially (away from goal) to exercise turning
	var brain: RobotBrain = body.get_node("Brain")
	await ticks(2)
	brain.setup_goto(arena.nav, Vector3(-5, 0, 8))
	brain.active = true
	var reached: bool = await ticks_until(func() -> bool: return brain.bb.goal_reached, 20 * 60)  # up to 20 s
	assert_bool(reached).append_failure_message("final pos %s" % body.global_position).is_true()
	assert_int(body.collision_count).is_less_equal(1)


func test_hider_leaves_spawn_and_hides() -> void:
	var r: Array = await NavArenaHelper.ready_arena(self)
	var arena: Arena = r[1]
	var grid := VisibilityGrid.new()
	var los := NavArenaHelper.los_walls(arena)
	grid.build(arena.bounds(), arena.wall_rects(), arena.seeker_spawn_point(), los)
	var spawn: Vector3 = arena.hider_spawn_points()[0]
	var body := NavArenaHelper.spawn_robot(arena, spawn, "hider")
	var brain: RobotBrain = body.get_node("Brain")
	var rng := RandomNumberGenerator.new()
	rng.seed = 1
	await ticks(2)
	brain.setup("hider", rng, arena.nav, grid, los, RoundConfig.new(), arena.seeker_spawn_point())
	brain.active = true
	await ticks(60)
	assert_str(brain.fsm.current_name()).is_equal("GoToSpot")
	var hid: bool = await ticks_until(func() -> bool: return brain.fsm.current_name() == "Hide", 30 * 60)  # up to 30 s
	assert_bool(hid).append_failure_message("state %s at %s" % [brain.fsm.current_name(), body.global_position]).is_true()
	assert_float(body.global_position.distance_to(spawn)).is_greater(2.0)
	assert_bool(grid.visible_from_spawn[grid.nearest_cell(body.global_position)]).is_false()


func test_seeker_waits_until_allowed_then_patrols() -> void:
	var r: Array = await NavArenaHelper.ready_arena(self)
	var arena: Arena = r[1]
	var grid := VisibilityGrid.new()
	var los := NavArenaHelper.los_walls(arena)
	grid.build(arena.bounds(), arena.wall_rects(), arena.seeker_spawn_point(), los)
	var body := NavArenaHelper.spawn_robot(arena, arena.seeker_spawn_point(), "seeker")
	var brain: RobotBrain = body.get_node("Brain")
	var rng := RandomNumberGenerator.new()
	rng.seed = 2
	await ticks(60)  # settle onto the floor first: the ~0.2 m drop is not "moving"
	brain.setup("seeker", rng, arena.nav, grid, los, RoundConfig.new(), arena.seeker_spawn_point())
	brain.active = true
	var start: Vector3 = body.global_position
	await ticks(120)  # 2 s in Wait
	assert_str(brain.fsm.current_name()).is_equal("Wait")
	assert_float(_xz(body.global_position).distance_to(_xz(start))).is_less(0.1)
	brain.bb.seeking_allowed = true
	await ticks(180)  # 3 s of Patrol
	assert_str(brain.fsm.current_name()).is_equal("Patrol")
	assert_float(_xz(body.global_position).distance_to(_xz(start))).is_greater(1.0)
	assert_float(body.distance_travelled).is_greater(1.0)


func test_sensor_ablation_flags_reach_blackboard() -> void:
	var r: Array = await NavArenaHelper.ready_arena(self)
	var arena: Arena = r[1]
	var cfg := RoundConfig.new()
	cfg.seeker_sensors = PackedStringArray(["lidar", "bumper"])
	var body := NavArenaHelper.spawn_robot(arena, arena.seeker_spawn_point(), "seeker")
	var brain: RobotBrain = body.get_node("Brain")
	await ticks(2)
	brain.setup("seeker", RandomNumberGenerator.new(), arena.nav, VisibilityGrid.new(), NavArenaHelper.los_walls(arena), cfg, arena.seeker_spawn_point())
	brain.active = true
	await ticks(3)
	assert_bool(brain.bb.lidar_enabled).is_true()
	assert_bool(brain.bb.vision_enabled).is_false()
	assert_bool(brain.bb.ultrasonic_enabled).is_false()
	assert_bool(brain.bb.bumper_enabled).is_true()


func test_setup_gives_each_robot_its_own_seeded_rng_and_vision_noise() -> void:
	var r: Array = await NavArenaHelper.ready_arena(self)
	var arena: Arena = r[1]
	var cfg := RoundConfig.new()
	cfg.noise_std = 0.2
	cfg.bearing_noise_deg = 4.0
	var a := NavArenaHelper.spawn_robot(arena, arena.seeker_spawn_point(), "seeker")
	var b := NavArenaHelper.spawn_robot(arena, arena.hider_spawn_points()[0], "hider")
	await ticks(2)
	var round_rng := RandomNumberGenerator.new()
	round_rng.seed = 9
	var brain_a: RobotBrain = a.get_node("Brain")
	var brain_b: RobotBrain = b.get_node("Brain")
	var grid := VisibilityGrid.new()
	var los := NavArenaHelper.los_walls(arena)
	grid.build(arena.bounds(), arena.wall_rects(), arena.seeker_spawn_point(), los)
	brain_a.setup("seeker", round_rng, arena.nav, grid, los, cfg, arena.seeker_spawn_point())
	brain_b.setup("hider", round_rng, arena.nav, grid, los, cfg, arena.seeker_spawn_point())
	assert_object(brain_a.bb.rng).is_not_same(round_rng)
	assert_object(brain_a.bb.rng).is_not_same(brain_b.bb.rng)
	# Seeded from the round RNG in setup order.
	var expect := RandomNumberGenerator.new()
	expect.seed = 9
	assert_int(brain_a.bb.rng.seed).is_equal(expect.randi())
	assert_int(brain_b.bb.rng.seed).is_equal(expect.randi())
	var vision: VisionCone = a.get_node("Sensors/Vision")
	assert_float(vision.noise_std).is_equal(0.2)
	assert_float(vision.bearing_noise_std).is_equal_approx(deg_to_rad(4.0), 1e-6)


func test_set_caught_stops_robot() -> void:
	var r: Array = await NavArenaHelper.ready_arena(self)
	var arena: Arena = r[1]
	var body := NavArenaHelper.spawn_robot(arena, Vector3(-13, 0, 8), "hider")
	var brain: RobotBrain = body.get_node("Brain")
	await ticks(2)
	brain.setup_goto(arena.nav, Vector3(-5, 0, 8))
	brain.active = true
	await ticks(60)
	brain.set_caught()
	await ticks(30)
	var p: Vector3 = body.global_position
	await ticks(30)
	assert_float(_xz(body.global_position).distance_to(_xz(p))).is_less(0.05)
	assert_str(brain.stats()["state"]).is_equal("Caught")


func test_inactive_brain_holds_body_still() -> void:
	var r: Array = await NavArenaHelper.ready_arena(self)
	var arena: Arena = r[1]
	var body := NavArenaHelper.spawn_robot(arena, Vector3(-13, 0, 8), "hider")
	var brain: RobotBrain = body.get_node("Brain")
	await ticks(2)
	brain.setup_goto(arena.nav, Vector3(-5, 0, 8))
	brain.active = true
	await ticks(60)  # driving toward the goal
	brain.active = false  # frozen: the brain must zero the latched DriveBody command
	await ticks(30)  # wheel deceleration is rate-limited: <= 20 ticks from full speed
	var p: Vector3 = body.global_position
	await ticks(30)
	assert_float(_xz(body.global_position).distance_to(_xz(p))).is_less(0.05)
