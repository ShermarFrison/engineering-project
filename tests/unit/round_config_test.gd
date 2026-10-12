extends GdUnitTestSuite


func test_defaults() -> void:
	var c := RoundConfig.new()
	assert_int(c.hider_count).is_equal(2)
	assert_bool(c.has_sensor("seeker", "vision")).is_true()
	assert_bool(c.has_sensor("hider", "lidar")).is_true()
	assert_str(c.sensors_label("seeker")).is_equal("lidar+vision+ultrasonic+bumper")


func test_ablation_removes_sensor() -> void:
	var c := RoundConfig.new()
	c.seeker_sensors = PackedStringArray(["lidar", "ultrasonic", "bumper"])
	assert_bool(c.has_sensor("seeker", "vision")).is_false()
	assert_bool(c.has_sensor("hider", "vision")).is_true()
	assert_str(c.sensors_label("seeker")).is_equal("lidar+ultrasonic+bumper")


func test_duplicate_is_independent() -> void:
	var a := RoundConfig.new()
	var b: RoundConfig = a.duplicate()
	b.seed = 5
	assert_int(a.seed).is_equal(0)
