extends GdUnitTestSuite


func _bb_with_lidar(ranges: Array) -> Blackboard:
	var bb := Blackboard.new()
	bb.lidar_enabled = true
	bb.lidar_max_range = 8.0
	var n: int = ranges.size()
	for i: int in n:
		bb.lidar_ranges.append(ranges[i])
		bb.lidar_angles.append(wrapf(TAU * i / n, -PI, PI))
	return bb


func test_nearest_obstacle_picks_min_range() -> void:
	var bb := _bb_with_lidar([8, 8, 2.5, 8, 8, 8, 8, 8])
	var o: Dictionary = bb.nearest_obstacle()
	assert_float(o["distance"]).is_equal_approx(2.5, 1e-6)
	assert_float(o["angle"]).is_equal_approx(TAU * 2 / 8, 1e-6)


func test_nearest_obstacle_inf_when_nothing_hit() -> void:
	var bb := _bb_with_lidar([8, 8, 8, 8])
	assert_bool(is_inf(bb.nearest_obstacle()["distance"])).is_true()


func test_min_range_in_arc_front() -> void:
	# angles: 0, 45, 90, 135, 180, -135, -90, -45 deg
	var bb := _bb_with_lidar([5, 8, 8, 8, 1, 8, 8, 3])
	assert_float(bb.min_range_in_arc(0.0, deg_to_rad(50))).is_equal_approx(3.0, 1e-6)
	assert_float(bb.min_range_in_arc(PI, deg_to_rad(10))).is_equal_approx(1.0, 1e-6)


func test_local_world_angle_roundtrip() -> void:
	var bb := Blackboard.new()
	bb.position = Vector3(1, 0, 2)
	bb.yaw = 0.3
	var dir: Vector3 = bb.local_to_world_dir(0.5)
	var p: Vector3 = bb.position + dir * 3.0
	assert_float(bb.world_to_local_angle(p)).is_equal_approx(0.5, 1e-5)


func test_forward_is_minus_z_at_zero_yaw() -> void:
	var bb := Blackboard.new()
	assert_vector(bb.local_to_world_dir(0.0)).is_equal_approx(Vector3(0, 0, -1), Vector3(1e-6, 1e-6, 1e-6))
	# positive angle turns left (toward -X)
	assert_float(bb.local_to_world_dir(0.5).x).is_less(0.0)


func test_seen_hiders_excludes_caught_and_seekers() -> void:
	var bb := Blackboard.new()
	bb.caught_ids = [2]
	bb.seen_robots = [
		{"id": 1, "role": "hider", "position": Vector3.ZERO, "bearing": 0.0, "distance": 1.0, "time": 0.0},
		{"id": 2, "role": "hider", "position": Vector3.ZERO, "bearing": 0.0, "distance": 1.0, "time": 0.0},
		{"id": 3, "role": "seeker", "position": Vector3.ZERO, "bearing": 0.0, "distance": 1.0, "time": 0.0},
	]
	var h := bb.seen_hiders()
	assert_int(h.size()).is_equal(1)
	assert_int(h[0]["id"]).is_equal(1)
	assert_int(bb.seen_seeker()["id"]).is_equal(3)


func test_time_since_seeker_seen() -> void:
	var bb := Blackboard.new()
	bb.time = 10.0
	assert_bool(is_inf(bb.time_since_seeker_seen())).is_true()
	bb.seeker_last_seen_time = 7.5
	assert_float(bb.time_since_seeker_seen()).is_equal_approx(2.5, 1e-6)
