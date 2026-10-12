extends GdUnitTestSuite


func _res(caught: Array[bool]) -> Dictionary:
	var hiders: Array[Dictionary] = []
	for i: int in caught.size():
		hiders.append({"index": i, "caught": caught[i], "catch_time": 12.5 if caught[i] else -1.0,
			"distance": 10.0 + i, "collisions": i, "stuck_events": 0,
			"transitions": [{"time": 1.0, "from": "PickSpot", "to": "GoToSpot", "reason": "spot chosen"}],
			"transition_counts": {"PickSpot->GoToSpot": 1}})
	var n_caught: int = 0
	for c: bool in caught:
		if c:
			n_caught += 1
	return {
		"variant": "all", "seed": 4, "hider_strategy": "static",
		"seeker_sensors": "lidar+vision+ultrasonic+bumper", "hider_sensors": "lidar+vision+ultrasonic+bumper",
		"hiders_total": caught.size(), "hiders_caught": n_caught, "duration": 60.0,
		"seeker": {"distance": 42.0, "collisions": 1, "stuck_events": 2,
			"transitions": [{"time": 15.0, "from": "Wait", "to": "Patrol", "reason": "head start over"},
				{"time": 20.0, "from": "Patrol", "to": "Chase", "reason": "hider seen"}],
			"transition_counts": {"Wait->Patrol": 1, "Patrol->Chase": 1}},
		"hiders": hiders,
	}


func test_header_and_row_have_same_length() -> void:
	var row := ResultsCsv.row(_res([true, false]))
	assert_int(row.size()).is_equal(ResultsCsv.header().size())


func test_row_values() -> void:
	var h := ResultsCsv.header()
	var row := ResultsCsv.row(_res([true, false]))
	assert_str(row[h.find("variant")]).is_equal("all")
	assert_str(row[h.find("seed")]).is_equal("4")
	assert_str(row[h.find("hiders_caught")]).is_equal("1")
	assert_str(row[h.find("duration")]).is_equal("60.000")
	assert_str(row[h.find("seeker_distance")]).is_equal("42.000")
	assert_str(row[h.find("seeker_transitions")]).is_equal("2")
	assert_str(row[h.find("hider0_caught")]).is_equal("1")
	assert_str(row[h.find("hider0_catch_time")]).is_equal("12.500")
	assert_str(row[h.find("hider1_caught")]).is_equal("0")


func test_csv_row_with_survivors() -> void:
	var h := ResultsCsv.header()
	var row := ResultsCsv.row(_res([false]))
	assert_str(row[h.find("hider0_catch_time")]).is_equal("-1.000")
	assert_str(row[h.find("hider2_caught")]).is_equal("")
	assert_str(row[h.find("hider2_distance")]).is_equal("")


func test_transition_rows() -> void:
	var rows := ResultsCsv.transitions_rows(_res([true]))
	assert_int(rows.size()).is_equal(3)  # 2 seeker + 1 hider
	assert_int(ResultsCsv.transitions_header().size()).is_equal(7)
	assert_array(Array(rows[0])).is_equal(["all", "4", "seeker", "15.000", "Wait", "Patrol", "head start over"])
	assert_array(Array(rows[2])).is_equal(["all", "4", "hider0", "1.000", "PickSpot", "GoToSpot", "spot chosen"])
