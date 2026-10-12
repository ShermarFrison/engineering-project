extends PhysicsTestSuite

const ROBOT := preload("res://scenes/robots/robot.tscn")
const FLOOR := preload("res://tests/fixtures/flat_floor.tscn")


func test_lidar_reads_wall_distance_and_max_range_elsewhere() -> void:
	var runner := scene_runner(auto_free(FLOOR.instantiate()))
	var robot: DriveBody = ROBOT.instantiate()
	robot.position = Vector3(0, 0.3, 0)
	runner.scene().add_child(robot)
	await ticks(30)  # settle on the floor
	var lidar: Lidar = robot.get_node("Sensors/Lidar")
	lidar.body = robot
	var bb := Blackboard.new()
	bb.position = robot.global_position
	bb.yaw = robot.yaw()
	lidar.sense(bb)
	assert_bool(bb.lidar_enabled).is_true()
	assert_int(bb.lidar_ranges.size()).is_equal(36)
	# Wall at x=+5 is to the robot's right (-90 deg) when facing -Z.
	var right_idx: int = -1
	for i: int in bb.lidar_angles.size():
		if absf(wrapf(bb.lidar_angles[i] + PI / 2, -PI, PI)) < 1e-4:
			right_idx = i
	assert_int(right_idx).is_not_equal(-1)
	# Wall face: centre 5.0 minus half thickness, measured from the lidar at the robot's
	# centre. The origin is inside our own mast; the excluded RID keeps it out of the reading.
	assert_float(bb.lidar_ranges[right_idx]).is_equal_approx(4.9, 0.1)
	# Forward (-Z) is open for 30 m: reads max_range.
	assert_float(bb.lidar_ranges[0]).is_equal_approx(lidar.max_range, 1e-3)


func test_disabled_lidar_leaves_blackboard_empty() -> void:
	var runner := scene_runner(auto_free(FLOOR.instantiate()))
	var robot: DriveBody = ROBOT.instantiate()
	runner.scene().add_child(robot)
	await ticks(2)
	var lidar: Lidar = robot.get_node("Sensors/Lidar")
	lidar.body = robot
	lidar.enabled = false
	var bb := Blackboard.new()
	lidar.sense(bb)
	assert_bool(bb.lidar_enabled).is_false()
	assert_int(bb.lidar_ranges.size()).is_equal(0)


func test_noisy_ranges_stay_within_zero_and_max_range() -> void:
	var runner := scene_runner(auto_free(FLOOR.instantiate()))
	var robot: DriveBody = ROBOT.instantiate()
	robot.position = Vector3(4.6, 0.3, 0)  # 0.3 m from the wall at x = 4.9
	runner.scene().add_child(robot)
	await ticks(30)
	var lidar: Lidar = robot.get_node("Sensors/Lidar")
	lidar.body = robot
	lidar.noise_std = 3.0
	var bb := Blackboard.new()
	bb.rng.seed = 11
	bb.position = robot.global_position
	bb.yaw = robot.yaw()
	for k: int in 20:
		lidar.sense(bb)
		for r: float in bb.lidar_ranges:
			assert_float(r).is_between(0.0, lidar.max_range)
