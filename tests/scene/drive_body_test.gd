extends PhysicsTestSuite

const ROBOT := preload("res://scenes/robots/robot.tscn")
const FLOOR := preload("res://tests/fixtures/flat_floor.tscn")


func _spawn(runner: GdUnitSceneRunner, pos: Vector3) -> DriveBody:
	var robot: DriveBody = ROBOT.instantiate()
	robot.position = pos
	runner.scene().add_child(robot)
	return robot


func _xz(p: Vector3) -> Vector2:
	return Vector2(p.x, p.z)


func test_forward_command_moves_along_minus_z() -> void:
	var runner := scene_runner(auto_free(FLOOR.instantiate()))
	var robot := _spawn(runner, Vector3(0, 0.3, 0))
	await ticks(60)  # settle on the floor (the 0.3 m spawn drops ~0.2 m in ~12 ticks)
	var start: Vector3 = robot.global_position
	robot.set_command(1.0, 0.0)
	await ticks(60)  # 1 s at 60 Hz; the wheel ramp-up costs ~0.08 m
	var moved: Vector3 = robot.global_position - start
	assert_float(moved.z).is_between(-1.1, -0.8)
	assert_float(absf(moved.x)).is_less(0.05)
	assert_float(robot.distance_travelled).is_between(0.8, 1.1)


func test_turn_command_rotates_in_place() -> void:
	var runner := scene_runner(auto_free(FLOOR.instantiate()))
	var robot := _spawn(runner, Vector3(0, 0.3, 0))
	await ticks(60)  # settle first: the settling drop must not count as movement
	var start: Vector3 = robot.global_position
	robot.set_command(0.0, 1.0)
	await ticks(60)
	assert_float(robot.yaw()).is_between(0.9, 1.1)
	assert_float(_xz(robot.global_position).distance_to(_xz(start))).is_less(0.05)


func test_stop_command_holds_still() -> void:
	var runner := scene_runner(auto_free(FLOOR.instantiate()))
	var robot := _spawn(runner, Vector3(0, 0.3, 0))
	await ticks(60)
	robot.set_command(1.0, 0.0)
	await ticks(30)
	robot.set_command(0.0, 0.0)
	await ticks(30)
	var p: Vector3 = robot.global_position
	await ticks(30)
	assert_float(_xz(robot.global_position).distance_to(_xz(p))).is_less(0.02)


func test_contacts_and_collision_count_when_driving_into_wall() -> void:
	var runner := scene_runner(auto_free(FLOOR.instantiate()))
	var robot := _spawn(runner, Vector3(3.5, 0.3, 0))
	await ticks(60)
	robot.rotation.y = -PI / 2  # face +X toward the wall at x=5
	robot.set_command(1.5, 0.0)
	await ticks(120)  # 2 s; the 1.15 m gap (front 3.75 to wall face 4.9) closes after ~0.85 s
	assert_int(robot.collision_count).is_greater_equal(1)
	assert_array(robot.contacts).is_not_empty()
	var n: Vector3 = robot.contacts[0]["normal"]
	assert_float(n.x).is_less(-0.9)  # wall pushes us back toward -X


func test_robot_is_in_robots_group_and_layer_3() -> void:
	var runner := scene_runner(auto_free(FLOOR.instantiate()))
	var robot := _spawn(runner, Vector3(0, 0.3, 0))
	await ticks(1)
	assert_bool(robot.is_in_group("robots")).is_true()
	assert_int(robot.collision_layer).is_equal(4)


func test_wheel_meshes_spin_when_driving() -> void:
	var runner := scene_runner(auto_free(FLOOR.instantiate()))
	var robot := _spawn(runner, Vector3(0, 0.3, 0))
	await ticks(60)
	var wheel: MeshInstance3D = robot.get_node("WheelL")
	var before: Basis = wheel.transform.basis
	robot.set_command(1.0, 0.0)
	await ticks(10)
	assert_bool(wheel.transform.basis.is_equal_approx(before)).is_false()
	assert_vector(wheel.position).is_equal_approx(Vector3(-0.2, 0.011, 0.0), Vector3(1e-4, 1e-4, 1e-4))  # spins in place


## Robot-frame direction of the rim point that was at the bottom of the wheel at rest.
func _rest_bottom_dir(wheel: Node3D, rest: Basis) -> Vector3:
	var p_local: Vector3 = rest.inverse() * Vector3.DOWN
	return wheel.transform.basis * p_local


func test_wheels_roll_forward_when_driving_forward() -> void:
	var runner := scene_runner(auto_free(FLOOR.instantiate()))
	var robot := _spawn(runner, Vector3(0, 0.3, 0))
	await ticks(60)
	var wl: Node3D = robot.get_node("WheelL")
	var wr: Node3D = robot.get_node("WheelR")
	var rest_l: Basis = wl.transform.basis
	var rest_r: Basis = wr.transform.basis
	robot.set_command(1.0, 0.0)
	await ticks(6)  # small spin (< 1 rad) so the direction is unambiguous
	# Rolling toward -Z without slip: the bottom of each wheel moves backward (+Z) and up.
	assert_float(_rest_bottom_dir(wl, rest_l).z).is_greater(0.05)
	assert_float(_rest_bottom_dir(wr, rest_r).z).is_greater(0.05)


func test_wheel_meshes_touch_the_floor_at_rest() -> void:
	var runner := scene_runner(auto_free(FLOOR.instantiate()))
	var robot := _spawn(runner, Vector3(0, 0.3, 0))
	await ticks(60)
	for n: String in ["WheelL", "WheelR"]:
		var wheel: MeshInstance3D = robot.get_node(n)
		var radius: float = (wheel.mesh as CylinderMesh).top_radius
		var bottom: float = wheel.global_position.y - radius  # axle is horizontal
		assert_float(bottom).append_failure_message("%s bottom at %.3f" % [n, bottom]).is_between(-0.01, 0.01)
