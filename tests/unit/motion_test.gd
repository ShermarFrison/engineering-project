extends GdUnitTestSuite


func _straight(from: Vector3, to: Vector3) -> PackedVector3Array:
	return PackedVector3Array([from, to])


func _bb(pos: Vector3, yaw: float, goal: Vector3) -> Blackboard:
	var bb := Blackboard.new()
	bb.position = pos
	bb.yaw = yaw
	bb.goal_pos = goal
	bb.goal_active = true
	return bb


## Integrate (v, w) into the blackboard pose for dt (pure kinematics).
func _step(bb: Blackboard, cmd: Vector2, dt: float) -> void:
	bb.yaw = wrapf(bb.yaw + cmd.y * dt, -PI, PI)
	bb.position += bb.local_to_world_dir(0.0) * cmd.x * dt
	bb.time += dt


func test_reaches_goal_in_open_space() -> void:
	var m := Motion.new()
	m.path_provider = _straight
	var bb := _bb(Vector3.ZERO, 0.0, Vector3(5, 0, -5))
	var reached: bool = false
	for i: int in 600:  # 10 s
		var cmd: Vector2 = m.update(bb)
		if bb.goal_reached:
			reached = true
			break
		_step(bb, cmd, 1.0 / 60.0)
	assert_bool(reached).is_true()
	assert_float(bb.position.distance_to(Vector3(5, 0, -5))).is_less(m.arrive_radius + 0.05)


func test_turns_toward_goal_first() -> void:
	var m := Motion.new()
	m.path_provider = _straight
	var bb := _bb(Vector3.ZERO, 0.0, Vector3(5, 0, 0))  # goal to the right (+X) => negative w
	var cmd: Vector2 = m.update(bb)
	assert_float(cmd.y).is_less(0.0)
	assert_float(cmd.x).is_less(m.max_v * 0.5)  # slow while turning


func test_inactive_goal_stops() -> void:
	var m := Motion.new()
	m.path_provider = _straight
	var bb := _bb(Vector3.ZERO, 0.0, Vector3(5, 0, 0))
	bb.goal_active = false
	assert_vector(m.update(bb)).is_equal(Vector2.ZERO)


func test_stuck_flag_when_no_progress() -> void:
	var m := Motion.new()
	m.path_provider = _straight
	var bb := _bb(Vector3.ZERO, 0.0, Vector3(0, 0, -5))
	var stuck_seen: bool = false
	for i: int in 400:  # 6.7 s without moving the pose
		m.update(bb)
		bb.time += 1.0 / 60.0
		if bb.stuck:
			stuck_seen = true
			break
	assert_bool(stuck_seen).is_true()
	assert_float(bb.time).is_between(m.stuck_window - 0.1, m.stuck_window + 1.0)


func test_no_stuck_flag_while_progressing() -> void:
	var m := Motion.new()
	m.path_provider = _straight
	var bb := _bb(Vector3.ZERO, 0.0, Vector3(0, 0, -20))
	for i: int in 300:
		var cmd: Vector2 = m.update(bb)
		assert_bool(bb.stuck).is_false()
		_step(bb, cmd, 1.0 / 60.0)


func test_bumper_triggers_reverse_recovery() -> void:
	var m := Motion.new()
	m.path_provider = _straight
	var bb := _bb(Vector3.ZERO, 0.0, Vector3(0, 0, -5))
	bb.bumper_enabled = true
	bb.bumper_hit = true
	bb.bumper_normal = Vector3(0, 0, 1)  # wall ahead pushes us back (+Z)
	var cmd: Vector2 = m.update(bb)
	assert_float(cmd.x).is_less(0.0)
	assert_float(absf(cmd.y)).is_equal(1.5)
	bb.bumper_hit = false
	bb.time += 0.3
	cmd = m.update(bb)
	assert_float(cmd.x).is_less(0.0)  # still recovering
	bb.time += 1.0
	cmd = m.update(bb)
	assert_float(cmd.x).is_greater(0.0)  # recovery over


func test_ultrasonic_slows_forward_speed() -> void:
	var m := Motion.new()
	m.path_provider = _straight
	var bb := _bb(Vector3.ZERO, 0.0, Vector3(0, 0, -5))
	var fast: Vector2 = m.update(bb)
	bb.ultrasonic_enabled = true
	bb.ultrasonic = PackedFloat32Array([0.3, 1.2, 1.2])
	var slow: Vector2 = m.update(bb)
	assert_float(slow.x).is_less(fast.x * 0.5)


func test_lidar_obstacle_ahead_causes_turn() -> void:
	var m := Motion.new()
	m.path_provider = _straight
	var bb := _bb(Vector3.ZERO, 0.0, Vector3(0, 0, -5))
	bb.lidar_enabled = true
	bb.lidar_max_range = 8.0
	for i: int in 36:
		var a: float = wrapf(TAU * i / 36, -PI, PI)
		bb.lidar_angles.append(a)
		bb.lidar_ranges.append(1.0 if absf(a) < deg_to_rad(25) else 8.0)
	var cmd: Vector2 = m.update(bb)
	assert_float(absf(cmd.y)).is_greater(0.5)


func _bump_cmd(normal: Vector3) -> Vector2:
	var m := Motion.new()
	m.path_provider = _straight
	var bb := _bb(Vector3.ZERO, 0.0, Vector3(0, 0, -5))
	bb.bumper_enabled = true
	bb.bumper_hit = true
	bb.bumper_normal = normal
	return m.update(bb)


func test_recovery_turns_away_from_contact_left_normal() -> void:
	var cmd: Vector2 = _bump_cmd(Vector3(-1, 0, 0))  # n_angle = +PI/2
	assert_float(cmd.x).is_less(0.0)
	assert_float(cmd.y).is_equal(1.5)


func test_recovery_turns_away_from_contact_right_normal() -> void:
	var cmd: Vector2 = _bump_cmd(Vector3(1, 0, 0))  # n_angle = -PI/2
	assert_float(cmd.x).is_less(0.0)
	assert_float(cmd.y).is_equal(-1.5)


func _high_path(_from: Vector3, to: Vector3) -> PackedVector3Array:
	return PackedVector3Array([Vector3(0, 0.5, -2), Vector3(to.x, 0.5, to.z)])


func test_waypoints_with_y_offset_are_advanced_and_goal_reached() -> void:
	var m := Motion.new()
	m.path_provider = _high_path
	var bb := _bb(Vector3(0, 0.09, 0), 0.0, Vector3(0, 0.5, -4))
	var reached: bool = false
	for i: int in 600:
		var cmd: Vector2 = m.update(bb)
		if bb.goal_reached:
			reached = true
			break
		_step(bb, cmd, 1.0 / 60.0)
	assert_bool(reached).is_true()
	var d: float = Vector2(bb.position.x, bb.position.z).distance_to(Vector2(0, -4))
	assert_float(d).is_less(m.arrive_radius)
