extends GdUnitTestSuite


func test_due_every_tick_at_default_60_hz() -> void:
	var s: SensorBase = auto_free(SensorBase.new())
	for i: int in 10:
		assert_bool(s.due(1.0 / 60.0)).is_true()


func test_due_at_10_hz_samples_every_sixth_tick() -> void:
	var s: SensorBase = auto_free(SensorBase.new())
	s.update_hz = 10.0
	var samples: int = 0
	for i: int in 60:
		if s.due(1.0 / 60.0):
			samples += 1
	assert_int(samples).is_equal(10)
