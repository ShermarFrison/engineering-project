extends PhysicsTestSuite

const ROBOT := preload("res://scenes/robots/robot.tscn")
const FLOOR := preload("res://tests/fixtures/flat_floor.tscn")


func _spawn(runner: GdUnitSceneRunner, pos: Vector3, role: String, yaw: float = 0.0) -> DriveBody:
	var r: DriveBody = ROBOT.instantiate()
	r.position = pos
	r.rotation.y = yaw
	r.role = role
	runner.scene().add_child(r)
	return r


func _sense(seeker: DriveBody) -> Blackboard:
	var vision: VisionCone = seeker.get_node("Sensors/Vision")
	vision.body = seeker
	var bb := Blackboard.new()
	bb.time = 4.0
	bb.position = seeker.global_position
	bb.yaw = seeker.yaw()
	vision.sense(bb)
	return bb


func test_sees_robot_in_front_within_range() -> void:
	var runner := scene_runner(auto_free(FLOOR.instantiate()))
	var seeker := _spawn(runner, Vector3(0, 0.3, 0), "seeker")
	var hider := _spawn(runner, Vector3(0.5, 0.3, -6), "hider")
	await ticks(30)  # both robots settled on the floor
	# The LOS ray runs from our eye to the hider's mast centre (body + 0.25 m) and hits the mast.
	var bb := _sense(seeker)
	assert_int(bb.seen_robots.size()).is_equal(1)
	var s: Dictionary = bb.seen_robots[0]
	assert_int(s["id"]).is_equal(hider.get_instance_id())
	assert_str(s["role"]).is_equal("hider")
	assert_float(s["distance"]).is_equal_approx(6.02, 0.1)
	assert_float(s["bearing"]).is_less(0.0)  # slightly to the right (+X)
	assert_bool(bb.last_known.has(hider.get_instance_id())).is_true()


func test_does_not_see_robot_behind() -> void:
	var runner := scene_runner(auto_free(FLOOR.instantiate()))
	var seeker := _spawn(runner, Vector3(0, 0.3, 0), "seeker")
	_spawn(runner, Vector3(0, 0.3, 6), "hider")
	await ticks(30)  # both robots settled on the floor
	assert_array(_sense(seeker).seen_robots).is_empty()


func test_does_not_see_robot_beyond_range() -> void:
	var runner := scene_runner(auto_free(FLOOR.instantiate()))
	var seeker := _spawn(runner, Vector3(0, 0.3, 0), "seeker")
	_spawn(runner, Vector3(0, 0.3, -12), "hider")
	await ticks(30)  # both robots settled on the floor
	assert_array(_sense(seeker).seen_robots).is_empty()


func test_wall_blocks_line_of_sight() -> void:
	var runner := scene_runner(auto_free(FLOOR.instantiate()))
	# Seeker faces +X (yaw -90deg) toward the wall at x=5; hider behind it at x=7.
	var seeker := _spawn(runner, Vector3(2, 0.3, 0), "seeker", -PI / 2)
	_spawn(runner, Vector3(7, 0.3, 0), "hider")
	await ticks(30)  # both robots settled on the floor
	assert_array(_sense(seeker).seen_robots).is_empty()


func test_seeing_seeker_updates_seeker_memory() -> void:
	var runner := scene_runner(auto_free(FLOOR.instantiate()))
	var hider := _spawn(runner, Vector3(0, 0.3, 0), "hider")
	var seeker := _spawn(runner, Vector3(0, 0.3, -4), "seeker")
	await ticks(30)  # both robots settled on the floor
	var bb := _sense(hider)
	assert_float(bb.seeker_last_seen_time).is_equal_approx(4.0, 1e-6)
	assert_float(bb.seeker_last_seen_pos.distance_to(seeker.global_position)).is_less(0.01)


func test_disabled_vision_sees_nothing() -> void:
	var runner := scene_runner(auto_free(FLOOR.instantiate()))
	var seeker := _spawn(runner, Vector3(0, 0.3, 0), "seeker")
	_spawn(runner, Vector3(0, 0.3, -3), "hider")
	await ticks(30)  # both robots settled on the floor
	(seeker.get_node("Sensors/Vision") as VisionCone).enabled = false
	var bb := _sense(seeker)
	assert_bool(bb.vision_enabled).is_false()
	assert_array(bb.seen_robots).is_empty()


func _sense_noisy(seeker: DriveBody, dist_std: float, bearing_std: float) -> Blackboard:
	var vision: VisionCone = seeker.get_node("Sensors/Vision")
	vision.body = seeker
	vision.noise_std = dist_std
	vision.bearing_noise_std = bearing_std
	var bb := Blackboard.new()
	bb.rng.seed = 5
	bb.position = seeker.global_position
	bb.yaw = seeker.yaw()
	vision.sense(bb)
	return bb


func _xz_gap(a: Vector3, b: Vector3) -> float:
	return Vector2(a.x, a.z).distance_to(Vector2(b.x, b.z))


func test_zero_noise_reports_exact_position() -> void:
	var runner := scene_runner(auto_free(FLOOR.instantiate()))
	var seeker := _spawn(runner, Vector3(0, 0.3, 0), "seeker")
	var hider := _spawn(runner, Vector3(0.5, 0.3, -6), "hider")
	await ticks(30)
	var bb := _sense_noisy(seeker, 0.0, 0.0)
	assert_int(bb.seen_robots.size()).is_equal(1)
	assert_float(_xz_gap(bb.seen_robots[0]["position"], hider.global_position)).is_less(1e-4)
	assert_float(_xz_gap(bb.last_known[hider.get_instance_id()]["position"], hider.global_position)).is_less(1e-4)


func test_distance_noise_moves_the_reported_position() -> void:
	var runner := scene_runner(auto_free(FLOOR.instantiate()))
	var seeker := _spawn(runner, Vector3(0, 0.3, 0), "seeker")
	var hider := _spawn(runner, Vector3(0.5, 0.3, -6), "hider")
	await ticks(30)
	var bb := _sense_noisy(seeker, 0.5, 0.0)
	assert_int(bb.seen_robots.size()).is_equal(1)
	var s: Dictionary = bb.seen_robots[0]
	assert_float(_xz_gap(s["position"], hider.global_position)).is_greater(1e-3)
	assert_float((s["position"] as Vector3).y).is_equal_approx(hider.global_position.y, 1e-4)
	# Consistent with the reported (noisy) distance and bearing.
	var rebuilt: Vector3 = bb.position + bb.local_to_world_dir(s["bearing"]) * float(s["distance"])
	assert_float(_xz_gap(s["position"], rebuilt)).is_less(1e-4)
	assert_float(_xz_gap(bb.last_known[hider.get_instance_id()]["position"], s["position"])).is_less(1e-6)


func test_bearing_noise_moves_the_reported_position() -> void:
	var runner := scene_runner(auto_free(FLOOR.instantiate()))
	var seeker := _spawn(runner, Vector3(0, 0.3, 0), "seeker")
	var hider := _spawn(runner, Vector3(0.5, 0.3, -6), "hider")
	await ticks(30)
	var bb := _sense_noisy(seeker, 0.0, deg_to_rad(10.0))
	var s: Dictionary = bb.seen_robots[0]
	assert_float(_xz_gap(s["position"], hider.global_position)).is_greater(1e-3)
	# Distance unchanged: the point moved along the circle around the seeker.
	assert_float(_xz_gap(s["position"], bb.position)).is_equal_approx(_xz_gap(hider.global_position, bb.position), 1e-3)
