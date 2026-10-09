extends PhysicsTestSuite

const ROBOT := preload("res://scenes/robots/robot.tscn")
const FLOOR := preload("res://tests/fixtures/flat_floor.tscn")
const CRATE := preload("res://scenes/props/crate.tscn")


func _robot(runner: GdUnitSceneRunner, pos: Vector3, yaw: float) -> DriveBody:
	var r: DriveBody = ROBOT.instantiate()
	r.position = pos
	r.rotation.y = yaw
	runner.scene().add_child(r)
	for s in r.get_node("Sensors").get_children():
		if s is SensorBase:
			(s as SensorBase).body = r
	return r


func _bb(r: DriveBody) -> Blackboard:
	var bb := Blackboard.new()
	bb.position = r.global_position
	bb.yaw = r.yaw()
	return bb


func test_ultrasonic_front_reads_crate_distance() -> void:
	var runner := scene_runner(auto_free(FLOOR.instantiate()))
	var crate: StaticBody3D = CRATE.instantiate()
	crate.position = Vector3(0, 0.5, -1.5)  # face at z=-1.0
	runner.scene().add_child(crate)
	var r := _robot(runner, Vector3(0, 0.3, 0), 0.0)
	await ticks(30)  # settle on the floor
	var bb := _bb(r)
	(r.get_node("Sensors/Ultrasonic") as Ultrasonic).sense(bb)
	assert_bool(bb.ultrasonic_enabled).is_true()
	# sensor sits 0.15 m ahead of the body origin, face at 1.0 => 0.85 (probe: 0.850)
	assert_float(bb.ultrasonic[0]).is_between(0.80, 0.90)
	# The side cones (centred at +/-35 deg) have inner rays at +/-20 deg that graze the
	# same crate at ~0.905 m (probe), so they read farther than the front, not clear.
	assert_float(bb.ultrasonic[1]).is_greater(bb.ultrasonic[0])
	assert_float(bb.ultrasonic[2]).is_greater(bb.ultrasonic[0])


func test_ultrasonic_side_cones() -> void:
	var runner := scene_runner(auto_free(FLOOR.instantiate()))
	var crate: StaticBody3D = CRATE.instantiate()
	crate.position = Vector3(-0.8, 0.5, -0.9)  # ahead-left
	runner.scene().add_child(crate)
	var r := _robot(runner, Vector3(0, 0.3, 0), 0.0)
	await ticks(30)
	var bb := _bb(r)
	(r.get_node("Sensors/Ultrasonic") as Ultrasonic).sense(bb)
	assert_float(bb.ultrasonic[1]).is_less(1.2)       # left sees it
	assert_float(bb.ultrasonic[2]).is_equal_approx(1.2, 1e-3)  # right does not


func test_bumper_fires_on_frontal_wall_contact_only() -> void:
	var runner := scene_runner(auto_free(FLOOR.instantiate()))
	var r := _robot(runner, Vector3(4.0, 0.3, 0), -PI / 2)  # facing +X toward wall at x=5
	await ticks(30)
	var bb := _bb(r)
	(r.get_node("Sensors/Bumper") as Bumper).sense(bb)
	assert_bool(bb.bumper_hit).is_false()
	r.set_command(1.0, 0.0)
	await ticks(90)  # 1.5 s; the 0.65 m gap (front 4.25 to wall face 4.9) closes after ~0.75 s
	bb = _bb(r)
	(r.get_node("Sensors/Bumper") as Bumper).sense(bb)
	assert_bool(bb.bumper_hit).is_true()
	assert_float(bb.bumper_normal.x).is_less(-0.9)


func test_bumper_ignores_rear_contact() -> void:
	var runner := scene_runner(auto_free(FLOOR.instantiate()))
	var r := _robot(runner, Vector3(4.0, 0.3, 0), PI / 2)  # facing -X, wall is behind
	await ticks(30)
	r.set_command(-1.0, 0.0)  # reverse into the wall
	await ticks(90)
	var bb := _bb(r)
	(r.get_node("Sensors/Bumper") as Bumper).sense(bb)
	assert_bool(bb.bumper_hit).is_false()
	assert_array(r.contacts).is_not_empty()  # we did touch it


func test_bumper_hits_crate() -> void:
	var runner := scene_runner(auto_free(FLOOR.instantiate()))
	var crate: StaticBody3D = CRATE.instantiate()
	crate.position = Vector3(0, 0.5, -1.5)  # face at z = -1.0; chassis front at z = -0.25
	runner.scene().add_child(crate)
	var r := _robot(runner, Vector3(0, 0.3, 0), 0.0)
	await ticks(30)
	r.set_command(1.0, 0.0)
	await ticks(90)  # the 0.75 m gap closes after ~0.85 s
	var bb := _bb(r)
	(r.get_node("Sensors/Bumper") as Bumper).sense(bb)
	assert_bool(bb.bumper_hit).is_true()
	assert_float(bb.bumper_normal.z).is_greater(0.9)  # the crate pushes us back toward +Z
