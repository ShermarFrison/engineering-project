extends GdUnitTestSuite


class Counter extends FSMState:
	var ticks: int = 0
	var entered: int = 0
	var exited: int = 0
	var go_to: String = ""
	var after: int = 0

	func _init(n: String, to: String = "", after_ticks: int = 0) -> void:
		name = n
		go_to = to
		after = after_ticks

	func enter(_bb: Blackboard) -> void:
		entered += 1
		ticks = 0

	func update(_bb: Blackboard, _dt: float) -> String:
		ticks += 1
		if go_to != "" and ticks >= after:
			reason = "after %d ticks" % after
			return go_to
		return ""

	func exit(_bb: Blackboard) -> void:
		exited += 1


func test_start_enters_initial_state() -> void:
	var fsm := FSM.new()
	var a := Counter.new("A")
	fsm.add(a)
	var bb := Blackboard.new()
	fsm.start("A", bb)
	assert_str(fsm.current_name()).is_equal("A")
	assert_int(a.entered).is_equal(1)
	assert_str(bb.state_name).is_equal("A")


func test_transition_calls_exit_enter_and_logs() -> void:
	var fsm := FSM.new()
	var a := Counter.new("A", "B", 3)
	var b := Counter.new("B")
	fsm.add(a)
	fsm.add(b)
	var bb := Blackboard.new()
	fsm.start("A", bb)
	for i in 3:
		bb.time = i * 0.1
		fsm.update(bb, 0.1)
	assert_str(fsm.current_name()).is_equal("B")
	assert_int(a.exited).is_equal(1)
	assert_int(b.entered).is_equal(1)
	assert_int(fsm.transitions.size()).is_equal(1)
	assert_str(fsm.transitions[0]["from"]).is_equal("A")
	assert_str(fsm.transitions[0]["to"]).is_equal("B")
	assert_float(fsm.transitions[0]["time"]).is_equal_approx(0.2, 1e-6)
	assert_str(fsm.transitions[0]["reason"]).is_equal("after 3 ticks")
	assert_str(a.reason).is_empty()  # cleared after logging
	assert_int(fsm.transition_counts["A->B"]).is_equal(1)


func test_returning_own_name_does_not_transition() -> void:
	var fsm := FSM.new()
	var a := Counter.new("A", "A", 1)
	fsm.add(a)
	var bb := Blackboard.new()
	fsm.start("A", bb)
	fsm.update(bb, 0.1)
	fsm.update(bb, 0.1)
	assert_int(a.entered).is_equal(1)
	assert_array(fsm.transitions).is_empty()


func test_force_transition() -> void:
	var fsm := FSM.new()
	fsm.add(Counter.new("A"))
	fsm.add(Counter.new("B"))
	var bb := Blackboard.new()
	fsm.start("A", bb)
	fsm.force("B", bb)
	assert_str(fsm.current_name()).is_equal("B")
	assert_int(fsm.transitions.size()).is_equal(1)
	assert_str(fsm.transitions[0]["reason"]).is_equal("forced")
