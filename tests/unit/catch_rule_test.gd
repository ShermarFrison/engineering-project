extends GdUnitTestSuite


func test_catch_when_in_range_visible_and_chasing() -> void:
	assert_bool(CatchRule.is_catch(1.0, 1.5, "Chase", true)).is_true()
	assert_bool(CatchRule.is_catch(1.0, 1.5, "Catch", true)).is_true()


func test_catch_rule_blocked_los() -> void:
	assert_bool(CatchRule.is_catch(1.0, 1.5, "Chase", false)).is_false()


func test_catch_requires_chase_state() -> void:
	assert_bool(CatchRule.is_catch(0.5, 1.5, "Patrol", true)).is_false()
	assert_bool(CatchRule.is_catch(0.5, 1.5, "Wait", true)).is_false()
	assert_bool(CatchRule.is_catch(0.5, 1.5, "Investigate", true)).is_false()


func test_out_of_range() -> void:
	assert_bool(CatchRule.is_catch(1.6, 1.5, "Chase", true)).is_false()


func test_exactly_at_tag_range_is_not_a_catch() -> void:
	# spec: distance < tag range
	assert_bool(CatchRule.is_catch(1.5, 1.5, "Chase", true)).is_false()
