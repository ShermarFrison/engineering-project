extends GdUnitTestSuite


func test_wheels_from_body_straight() -> void:
	var w: Vector2 = DriveModel.wheels_from_body(1.0, 0.0, 0.1, 0.4)
	assert_float(w.x).is_equal_approx(10.0, 1e-6)
	assert_float(w.y).is_equal_approx(10.0, 1e-6)


func test_wheels_from_body_turn_left_right_wheel_faster() -> void:
	var w: Vector2 = DriveModel.wheels_from_body(0.0, 1.0, 0.1, 0.4)
	# w = r(wr - wl)/L  =>  wr - wl = L/r = 4 ; symmetric => wl=-2, wr=2
	assert_float(w.x).is_equal_approx(-2.0, 1e-6)
	assert_float(w.y).is_equal_approx(2.0, 1e-6)


func test_roundtrip() -> void:
	var w: Vector2 = DriveModel.wheels_from_body(0.7, -0.3, 0.1, 0.4)
	var b: Vector2 = DriveModel.body_from_wheels(w.x, w.y, 0.1, 0.4)
	assert_float(b.x).is_equal_approx(0.7, 1e-6)
	assert_float(b.y).is_equal_approx(-0.3, 1e-6)


func test_command_clamps_wheel_speed() -> void:
	var m := DriveModel.new()
	m.max_wheel_speed = 10.0
	m.max_wheel_accel = 1e9
	m.command(100.0, 0.0, 0.1)
	assert_float(m.wl).is_equal_approx(10.0, 1e-6)
	assert_float(m.wr).is_equal_approx(10.0, 1e-6)


func test_command_rate_limits_acceleration() -> void:
	var m := DriveModel.new()
	m.max_wheel_accel = 10.0
	m.command(1.0, 0.0, 0.1)  # target wl = 10 rad/s, but only 1 rad/s change allowed in 0.1 s
	assert_float(m.wl).is_equal_approx(1.0, 1e-6)
	m.command(1.0, 0.0, 0.1)
	assert_float(m.wl).is_equal_approx(2.0, 1e-6)


func test_body_velocity_matches_command_when_unconstrained() -> void:
	var m := DriveModel.new()
	m.max_wheel_accel = 1e9
	m.command(0.5, 0.25, 0.016)
	var b: Vector2 = m.body_velocity()
	assert_float(b.x).is_equal_approx(0.5, 1e-6)
	assert_float(b.y).is_equal_approx(0.25, 1e-6)


func test_max_linear_speed() -> void:
	var m := DriveModel.new()
	m.wheel_radius = 0.1
	m.max_wheel_speed = 25.0
	assert_float(m.max_linear_speed()).is_equal_approx(2.5, 1e-6)
