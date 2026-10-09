extends GdUnitTestSuite

const BOUNDS := Rect2(-10, -10, 20, 20)
# One wall along the Z axis at x in [-0.15, 0.15], z in [-10, 10], splitting the arena.
const WALL := Rect2(-0.15, -10, 0.3, 20)


## LOS blocked iff the segment crosses x = 0.
func _los(a: Vector3, b: Vector3) -> bool:
	return signf(a.x) == signf(b.x) or absf(a.x) < 1e-6 or absf(b.x) < 1e-6


func _grid() -> VisibilityGrid:
	var g := VisibilityGrid.new()
	g.build(BOUNDS, [WALL], Vector3(-7, 0, 0), _los)
	return g


func test_cells_avoid_walls_and_cover_bounds() -> void:
	var g := _grid()
	assert_int(g.centers.size()).is_equal(100)  # 10 x 10; the wall (grown 0.5 m) covers no centre
	assert_bool(g.centers.has(Vector3(9, 0, 9))).is_true()  # far +X/+Z corner cell exists
	assert_bool(g.centers.has(Vector3(-9, 0, -9))).is_true()
	assert_int(g.wall_rects.size()).is_equal(1)
	for c in g.centers:
		assert_bool(WALL.grow(0.5).has_point(Vector2(c.x, c.z))).is_false()
		assert_bool(BOUNDS.has_point(Vector2(c.x, c.z))).is_true()


func test_visible_from_spawn_respects_wall() -> void:
	var g := _grid()
	for i in g.centers.size():
		var c: Vector3 = g.centers[i]
		assert_bool(g.visible_from_spawn[i]).is_equal(c.x < 0.0)


func test_openness_lower_next_to_wall() -> void:
	var g := _grid()
	var near: int = g.nearest_cell(Vector3(1, 0, 0))
	var far: int = g.nearest_cell(Vector3(7, 0, 0))
	assert_float(g.openness[near]).is_less(g.openness[far])
	assert_float(g.openness[far]).is_equal_approx(1.0, 1e-6)


func test_patrol_prefers_unviewed_cells() -> void:
	var g := _grid()
	var marked: int = g.mark_viewed(Vector3(-7, 0, 0), 0.0, TAU, 100.0, 10.0, _los)
	assert_int(marked).is_greater(0)
	var t: Vector3 = g.pick_patrol_target(Vector3(-7, 0, 0), 10.0)
	assert_float(t.x).is_greater(0.0)  # the unviewed half


func test_mark_viewed_respects_fov() -> void:
	var g := _grid()
	# facing -Z (yaw 0) with 90 deg fov from the far east side: only cells with z < pos.z get marked
	var pos := Vector3(7, 0, 8)
	g.mark_viewed(pos, 0.0, deg_to_rad(90), 100.0, 5.0, _los)
	for i in g.centers.size():
		if g.last_viewed[i] > 0.0:
			assert_float(g.centers[i].z).is_less(pos.z + 0.01)


func test_hiding_spot_is_hidden_and_far_from_threat() -> void:
	var g := _grid()
	var threat := Vector3(-7, 0, 0)
	var s: Vector3 = g.pick_hiding_spot(threat, 6.0)
	assert_bool(g.visible_from_spawn[g.nearest_cell(s)]).is_false()
	assert_float(s.distance_to(threat)).is_greater_equal(6.0)


func test_hiding_spot_relaxes_distance_if_needed() -> void:
	var g := _grid()
	var s: Vector3 = g.pick_hiding_spot(Vector3(0, 0, 0), 1000.0)
	assert_bool(g.centers.has(s)).is_true()


func test_flee_spot_moves_away_from_threat() -> void:
	var g := _grid()
	var from := Vector3(3, 0, 0)
	var threat := Vector3(1, 0, 0)
	var s: Vector3 = g.pick_flee_spot(from, threat)
	assert_float(s.distance_to(threat)).is_greater(from.distance_to(threat))


func test_pick_hiding_spots_best_first() -> void:
	var g := _grid()
	var threat := Vector3(-7, 0, 0)
	var spots: Array[Vector3] = g.pick_hiding_spots(threat, 6.0, 3)
	assert_int(spots.size()).is_equal(3)
	assert_vector(spots[0]).is_equal(g.pick_hiding_spot(threat, 6.0))
	assert_bool(spots[0] != spots[1] and spots[1] != spots[2] and spots[0] != spots[2]).is_true()
