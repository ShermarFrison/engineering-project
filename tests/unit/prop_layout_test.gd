extends GdUnitTestSuite

const BOUNDS := Rect2(-15, -15, 30, 30)
const SIZES: Array[Vector2] = [Vector2(1, 1), Vector2(1.5, 1), Vector2(2, 1), Vector2(1.2, 1.2)]


func _walls() -> Array[Rect2]:
	return [Rect2(-15.3, -15.3, 30.6, 0.3), Rect2(-1, -1, 2, 2), Rect2(-15, -5.15, 6, 0.3)]


func _rng(seed: int) -> RandomNumberGenerator:
	var r := RandomNumberGenerator.new()
	r.seed = seed
	return r


func _rect(p: Dictionary) -> Rect2:
	var s: Vector2 = p["size"] if p["rot"] == 0.0 else Vector2(p["size"].y, p["size"].x)
	return Rect2(p["pos"] - s * 0.5, s)


func test_generates_requested_count() -> void:
	var layout := PropLayout.generate(_rng(1), 12, BOUNDS, _walls(), [], SIZES)
	assert_int(layout.size()).is_equal(12)


func test_props_do_not_overlap_each_other_or_walls() -> void:
	var layout := PropLayout.generate(_rng(2), 12, BOUNDS, _walls(), [], SIZES, 1.5)
	for i in layout.size():
		for j in range(i + 1, layout.size()):
			assert_bool(PropLayout.overlaps(_rect(layout[i]), _rect(layout[j]), 1.5)).is_false()
		for w in _walls():
			assert_bool(PropLayout.overlaps(_rect(layout[i]), w, 1.5)).is_false()


func test_props_stay_inside_bounds() -> void:
	var layout := PropLayout.generate(_rng(3), 12, BOUNDS, _walls(), [], SIZES)
	for p in layout:
		assert_bool(BOUNDS.grow(-1.0).encloses(_rect(p))).is_true()


func test_keep_clear_points_respected() -> void:
	var clear: Array[Vector3] = [Vector3(-7, 0, 0), Vector3(12, 0, 12)]
	var layout := PropLayout.generate(_rng(4), 12, BOUNDS, _walls(), clear, SIZES)
	for p in layout:
		for c in clear:
			var cr := Rect2(Vector2(c.x, c.z) - Vector2(1, 1), Vector2(2, 2))
			assert_bool(PropLayout.overlaps(_rect(p), cr, 1.5)).is_false()


func test_same_seed_same_layout() -> void:
	var a := PropLayout.generate(_rng(7), 12, BOUNDS, _walls(), [], SIZES)
	var b := PropLayout.generate(_rng(7), 12, BOUNDS, _walls(), [], SIZES)
	assert_array(a).is_equal(b)


func test_different_seed_different_layout() -> void:
	var a := PropLayout.generate(_rng(7), 12, BOUNDS, _walls(), [], SIZES)
	var b := PropLayout.generate(_rng(8), 12, BOUNDS, _walls(), [], SIZES)
	assert_array(a).is_not_equal(b)


func test_spawner_terminates_on_crowded_arena() -> void:
	# Tiny arena, huge gap: cannot fit 12. Must return fewer, not hang.
	var layout := PropLayout.generate(_rng(5), 12, Rect2(-3, -3, 6, 6), [], [], SIZES, 3.0, 500)
	assert_int(layout.size()).is_less(12)


func test_overlaps_uses_gap() -> void:
	var a := Rect2(0, 0, 1, 1)
	var b := Rect2(2, 0, 1, 1)  # 1 m apart
	assert_bool(PropLayout.overlaps(a, b, 0.5)).is_false()
	assert_bool(PropLayout.overlaps(a, b, 1.5)).is_true()
