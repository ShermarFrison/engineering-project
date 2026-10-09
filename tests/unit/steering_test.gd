extends GdUnitTestSuite

const N := 36


func _scan(hits: Dictionary) -> Array:
	var ranges := PackedFloat32Array()
	var angles := PackedFloat32Array()
	for i in N:
		ranges.append(hits.get(i, 8.0))
		angles.append(wrapf(TAU * i / N, -PI, PI))
	return [ranges, angles]


func test_no_obstacles_heads_to_target() -> void:
	var s := _scan({})
	assert_float(Steering.pick_heading(0.4, s[0], s[1], 8.0)).is_equal_approx(0.4, 1e-6)


func test_empty_scan_heads_to_target() -> void:
	assert_float(Steering.pick_heading(-1.0, PackedFloat32Array(), PackedFloat32Array(), 8.0)).is_equal_approx(-1.0, 1e-6)


func test_obstacle_ahead_deviates() -> void:
	# obstacle at 1 m spanning -20..+20 deg (indices 0,1,2 and 34,35)
	var s := _scan({0: 1.0, 1: 1.0, 2: 1.0, 34: 1.0, 35: 1.0})
	var h: float = Steering.pick_heading(0.0, s[0], s[1], 8.0)
	assert_float(absf(h)).is_greater_equal(deg_to_rad(30.0))
	assert_float(absf(h)).is_less_equal(deg_to_rad(90.0))


func test_far_obstacle_ignored() -> void:
	var s := _scan({0: 6.0, 1: 6.0, 35: 6.0})
	assert_float(Steering.pick_heading(0.0, s[0], s[1], 8.0)).is_equal_approx(0.0, 1e-6)


func test_prefers_side_with_more_room() -> void:
	# obstacle ahead; right side (negative angles) also cluttered at 1.2 m, left open
	var hits := {0: 1.0, 1: 1.0, 35: 1.0}
	for i in range(27, 35):
		hits[i] = 1.2
	var s := _scan(hits)
	assert_float(Steering.pick_heading(0.0, s[0], s[1], 8.0)).is_greater(0.0)


func test_speed_scale() -> void:
	assert_float(Steering.speed_scale(INF)).is_equal_approx(1.0, 1e-6)
	assert_float(Steering.speed_scale(1.0)).is_equal_approx(0.5, 1e-6)
	assert_float(Steering.speed_scale(0.1)).is_equal_approx(0.2, 1e-6)
