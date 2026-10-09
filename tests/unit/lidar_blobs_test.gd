extends GdUnitTestSuite

const N := 72  # 5 degree resolution


func _scan(hits: Dictionary, n: int = N) -> Array:  # idx -> range ; returns [ranges, angles]
	var ranges := PackedFloat32Array()
	var angles := PackedFloat32Array()
	for i in n:
		ranges.append(hits.get(i, 8.0))
		angles.append(wrapf(TAU * i / n, -PI, PI))
	return [ranges, angles]


func test_robot_sized_cluster_is_a_blob() -> void:
	# 3 consecutive rays at 2 m: chord = 2 * 2 * sin(5deg) = 0.349 m (a 0.3-0.42 m mast)
	var s := _scan({10: 2.0, 11: 2.0, 12: 2.0})
	var blobs := LidarBlobs.detect(s[0], s[1], 8.0)
	assert_int(blobs.size()).is_equal(1)
	assert_float(blobs[0]["distance"]).is_equal_approx(2.0, 1e-3)
	assert_float(blobs[0]["angle"]).is_equal_approx(TAU * 11 / N, 1e-6)
	assert_float(blobs[0]["width"]).is_between(0.3, 0.4)


func test_long_wall_is_not_a_blob() -> void:
	var hits := {}
	for i in range(20, 40):
		hits[i] = 4.0
	var s := _scan(hits)
	assert_array(LidarBlobs.detect(s[0], s[1], 8.0)).is_empty()


func test_cluster_touching_wall_is_not_a_blob() -> void:
	# wall at 4 m from idx 20..40, then an object at 3.8 m right next to it (continuous)
	var hits := {}
	for i in range(20, 40):
		hits[i] = 4.0
	hits[40] = 3.8
	hits[41] = 3.8
	var s := _scan(hits)
	assert_array(LidarBlobs.detect(s[0], s[1], 8.0)).is_empty()


func test_object_in_front_of_wall_is_a_blob_if_gap_is_large() -> void:
	var hits := {}
	for i in range(20, 50):
		hits[i] = 6.0
	hits[30] = 2.0
	hits[31] = 2.0
	hits[32] = 2.0
	var s := _scan(hits)
	var blobs := LidarBlobs.detect(s[0], s[1], 8.0)
	assert_int(blobs.size()).is_equal(1)
	assert_float(blobs[0]["distance"]).is_equal_approx(2.0, 1e-3)


func test_wraparound_cluster_across_index_zero() -> void:
	# chord = 2 * 2.5 * sin(5deg) = 0.436 m <= max_width 0.5
	var s := _scan({71: 2.5, 0: 2.5, 1: 2.5})
	var blobs := LidarBlobs.detect(s[0], s[1], 8.0)
	assert_int(blobs.size()).is_equal(1)
	assert_float(absf(blobs[0]["angle"])).is_less(0.01)


func test_single_ray_is_a_blob() -> void:
	# A far mast covers a single ray: chord 0 m, still robot-sized (min_width 0).
	var s := _scan({10: 1.0})
	var blobs := LidarBlobs.detect(s[0], s[1], 8.0)
	assert_int(blobs.size()).is_equal(1)
	assert_float(blobs[0]["width"]).is_equal_approx(0.0, 1e-6)


func test_crate_face_at_close_range_is_not_a_blob() -> void:
	# A 1.0 m crate face at 1 m spans ~53 deg: 11 rays at 5 deg, chord = 2 * sin(25deg) = 0.845 m > 0.5.
	var hits := {}
	for i in range(8, 19):
		hits[i] = 1.0
	var s := _scan(hits)
	assert_array(LidarBlobs.detect(s[0], s[1], 8.0)).is_empty()


func test_mast_seen_by_36_ray_lidar_from_1_to_6_m() -> void:
	# The game lidar has 36 rays (10 deg). A 0.3 m mast covers two rays at 1 m
	# (chord 2 * 1 * sin(5deg) = 0.17 m) and one ray at 2-6 m; each must be a blob.
	var cases: Array[Dictionary] = [{2: 1.0, 3: 1.0}, {2: 2.0}, {2: 4.0}, {2: 6.0}]
	for hits in cases:
		var s := _scan(hits, 36)
		var blobs := LidarBlobs.detect(s[0], s[1], 8.0)
		assert_int(blobs.size()).append_failure_message("hits %s" % hits).is_equal(1)


func test_off_walls_drops_blob_on_a_known_wall() -> void:
	var bb := Blackboard.new()  # at the origin, facing -Z
	var blobs: Array[Dictionary] = [
		{"angle": 0.0, "distance": 3.0, "width": 0.0},     # world (0, 0, -3): the wall end
		{"angle": PI / 2, "distance": 2.0, "width": 0.0},  # world (-2, 0, 0): free space
	]
	var walls: Array[Rect2] = [Rect2(-0.15, -7.0, 0.3, 4.0)]  # 0.3 m wall ending at z = -3, seen end-on
	var kept := LidarBlobs.off_walls(blobs, bb, walls)
	assert_int(kept.size()).is_equal(1)
	assert_float(kept[0]["angle"]).is_equal_approx(PI / 2, 1e-6)
