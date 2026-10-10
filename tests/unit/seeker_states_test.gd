extends GdUnitTestSuite

const BOUNDS := Rect2(-10, -10, 20, 20)


func _los_clear(_a: Vector3, _b: Vector3) -> bool:
	return true


func _ctx() -> SeekerStates.Context:
	var ctx := SeekerStates.Context.new()
	ctx.grid = VisibilityGrid.new()
	ctx.grid.build(BOUNDS, [], Vector3(-7, 0, 0), _los_clear)
	ctx.los = _los_clear
	return ctx


func _bb(pos: Vector3 = Vector3.ZERO) -> Blackboard:
	var bb := Blackboard.new()
	bb.role = "seeker"
	bb.position = pos
	bb.vision_enabled = true
	bb.lidar_enabled = true
	bb.lidar_max_range = 8.0
	return bb


func _seen(id: int, pos: Vector3, bb: Blackboard) -> Dictionary:
	return {"id": id, "role": "hider", "position": pos, "bearing": bb.world_to_local_angle(pos),
		"distance": bb.position.distance_to(pos), "time": bb.time}


func _start(ctx: SeekerStates.Context, bb: Blackboard, state: String) -> FSM:
	var fsm := SeekerStates.build(ctx)
	fsm.start(state, bb)
	return fsm


func _tick(fsm: FSM, bb: Blackboard, seconds: float) -> void:
	var steps: int = int(round(seconds * 60.0))
	for i: int in steps:
		bb.time += 1.0 / 60.0
		fsm.update(bb, 1.0 / 60.0)


func test_wait_to_patrol_when_seeking_allowed() -> void:
	var bb := _bb()
	var fsm := _start(_ctx(), bb, "Wait")
	_tick(fsm, bb, 1.0)
	assert_str(fsm.current_name()).is_equal("Wait")
	assert_bool(bb.goal_active).is_false()
	bb.seeking_allowed = true
	_tick(fsm, bb, 0.1)
	assert_str(fsm.current_name()).is_equal("Patrol")
	assert_str(fsm.transitions[-1]["reason"]).is_equal("head start over")


func test_patrol_sets_goal_and_repicks_when_reached() -> void:
	var bb := _bb()
	var fsm := _start(_ctx(), bb, "Patrol")
	assert_bool(bb.goal_active).is_true()
	var first: Vector3 = bb.goal_pos
	bb.position = first
	bb.goal_reached = true
	_tick(fsm, bb, 0.1)
	assert_bool(bb.goal_pos.distance_to(first) > 1.0).is_true()


func test_patrol_repicks_goal_when_stuck() -> void:
	var bb := _bb()
	var fsm := _start(_ctx(), bb, "Patrol")
	var first: Vector3 = bb.goal_pos
	bb.stuck = true
	_tick(fsm, bb, 1.0 / 60.0)  # one tick
	assert_str(fsm.current_name()).is_equal("Patrol")
	assert_bool(bb.goal_pos.distance_to(first) > 1.0).is_true()


func test_patrol_to_chase_on_seen_hider() -> void:
	var bb := _bb()
	var fsm := _start(_ctx(), bb, "Patrol")
	bb.seen_robots = [_seen(42, Vector3(0, 0, -5), bb)]
	_tick(fsm, bb, 0.1)
	assert_str(fsm.current_name()).is_equal("Chase")
	assert_float(bb.goal_pos.distance_to(Vector3(0, 0, -5))).is_less(0.01)


func test_patrol_to_investigate_on_blob() -> void:
	var bb := _bb()
	bb.vision_enabled = false
	var fsm := _start(_ctx(), bb, "Patrol")
	bb.blobs = [{"angle": 0.0, "distance": 4.0, "width": 0.3}]  # beyond chase_blob_dist (3 m)
	_tick(fsm, bb, 0.1)
	assert_str(fsm.current_name()).is_equal("Investigate")
	assert_float(bb.goal_pos.distance_to(Vector3(0, 0, -4))).is_less(0.1)


func test_patrol_to_chase_on_near_blob_without_vision() -> void:
	var bb := _bb()
	bb.vision_enabled = false
	var fsm := _start(_ctx(), bb, "Patrol")
	bb.blobs = [{"angle": 0.0, "distance": 2.0, "width": 0.3}]  # within chase_blob_dist (3 m)
	_tick(fsm, bb, 0.1)
	assert_str(fsm.current_name()).is_equal("Chase")
	assert_str(fsm.transitions[-1]["reason"]).is_equal("near blob, no vision")
	assert_float(bb.goal_pos.distance_to(Vector3(0, 0, -2))).is_less(0.1)


func test_patrol_with_vision_investigates_near_blob() -> void:
	var bb := _bb()  # vision on: a blob alone is never a detection
	var fsm := _start(_ctx(), bb, "Patrol")
	bb.blobs = [{"angle": 0.0, "distance": 2.0, "width": 0.3}]
	_tick(fsm, bb, 0.1)
	assert_str(fsm.current_name()).is_equal("Investigate")


func test_patrol_ignores_blob_near_caught_hider() -> void:
	var bb := _bb()
	bb.caught_ids = [7]
	bb.last_known[7] = {"position": Vector3(0, 0, -4), "time": 0.0, "role": "hider"}
	var fsm := _start(_ctx(), bb, "Patrol")
	bb.blobs = [{"angle": 0.0, "distance": 4.0, "width": 0.3}]
	_tick(fsm, bb, 0.1)
	assert_str(fsm.current_name()).is_equal("Patrol")


func test_investigate_to_patrol_on_timeout() -> void:
	var ctx := _ctx()
	var bb := _bb()
	ctx.investigated.append({"pos": Vector3(0, 0, -4), "time": 0.0})
	var fsm := _start(ctx, bb, "Investigate")
	_tick(fsm, bb, ctx.investigate_timeout + 0.2)
	assert_str(fsm.current_name()).is_equal("Patrol")
	assert_str(fsm.transitions[-1]["reason"]).is_equal("timeout")


func test_investigate_to_patrol_after_arrival_dwell() -> void:
	var ctx := _ctx()
	var bb := _bb()
	ctx.investigated.append({"pos": Vector3(0, 0, -4), "time": 0.0})
	var fsm := _start(ctx, bb, "Investigate")
	bb.position = Vector3(0, 0, -4)
	bb.goal_reached = true
	_tick(fsm, bb, 1.0)
	assert_str(fsm.current_name()).is_equal("Investigate")  # still dwelling (1.5 s)
	_tick(fsm, bb, 1.0)
	assert_str(fsm.current_name()).is_equal("Patrol")
	assert_str(fsm.transitions[-1]["reason"]).is_equal("nothing found")


func test_investigate_to_patrol_when_stuck() -> void:
	var ctx := _ctx()
	var bb := _bb()
	ctx.investigated.append({"pos": Vector3(0, 0, -4), "time": 0.0})
	var fsm := _start(ctx, bb, "Investigate")
	_tick(fsm, bb, 0.5)
	assert_str(fsm.current_name()).is_equal("Investigate")
	bb.stuck = true
	_tick(fsm, bb, 1.0 / 60.0)  # one tick
	assert_str(fsm.current_name()).is_equal("Patrol")
	assert_str(fsm.transitions[-1]["reason"]).is_equal("stuck")


func test_investigate_to_chase_on_seen() -> void:
	var bb := _bb()
	var fsm := _start(_ctx(), bb, "Investigate")
	bb.seen_robots = [_seen(42, Vector3(2, 0, -3), bb)]
	_tick(fsm, bb, 0.1)
	assert_str(fsm.current_name()).is_equal("Chase")


func test_investigate_to_chase_on_near_blob_without_vision() -> void:
	var bb := _bb()
	bb.vision_enabled = false
	var fsm := _start(_ctx(), bb, "Patrol")
	bb.blobs = [{"angle": 0.0, "distance": 4.0, "width": 0.3}]
	_tick(fsm, bb, 0.1)
	assert_str(fsm.current_name()).is_equal("Investigate")
	# The seeker closed in on the same blob (world (0, 0, -4)); it is now inside
	# chase_blob_dist. The investigate cooldown must not hide it from the chase check.
	bb.position = Vector3(0, 0, -1.5)
	bb.blobs = [{"angle": 0.0, "distance": 2.5, "width": 0.3}]
	_tick(fsm, bb, 0.1)
	assert_str(fsm.current_name()).is_equal("Chase")
	assert_float(bb.goal_pos.distance_to(Vector3(0, 0, -4))).is_less(0.1)


func test_chase_follows_and_catches_in_range() -> void:
	var ctx := _ctx()
	var bb := _bb()
	var fsm := _start(ctx, bb, "Chase")
	bb.seen_robots = [_seen(42, Vector3(0, 0, -3), bb)]
	_tick(fsm, bb, 0.1)
	assert_str(fsm.current_name()).is_equal("Chase")
	assert_float(bb.goal_pos.distance_to(Vector3(0, 0, -3))).is_less(0.01)
	bb.seen_robots = [_seen(42, Vector3(0, 0, -1.0), bb)]
	_tick(fsm, bb, 0.05)
	assert_str(fsm.current_name()).is_equal("Catch")
	assert_int(bb.catch_request).is_equal(42)


func test_chase_to_patrol_after_lose_timeout() -> void:
	var ctx := _ctx()
	var bb := _bb()
	var fsm := _start(ctx, bb, "Chase")
	bb.seen_robots = [_seen(42, Vector3(0, 0, -3), bb)]
	_tick(fsm, bb, 0.1)
	bb.seen_robots = []
	_tick(fsm, bb, ctx.lose_timeout - 0.5)
	assert_str(fsm.current_name()).is_equal("Chase")
	_tick(fsm, bb, 1.0)
	assert_str(fsm.current_name()).is_equal("Patrol")
	assert_int(fsm.transition_counts.get("Chase->Patrol", 0)).is_equal(1)
	assert_str(fsm.transitions[-1]["reason"]).is_equal("target lost")


func test_catch_to_patrol_after_delay_and_clears_request() -> void:
	var ctx := _ctx()
	var bb := _bb()
	bb.catch_request = 42
	ctx.chase_last_pos = Vector3(0, 0, -1.2)
	var fsm := _start(ctx, bb, "Catch")
	# The seeker keeps closing on the target during Catch.
	assert_bool(bb.goal_active).is_true()
	assert_float(bb.goal_pos.distance_to(Vector3(0, 0, -1.2))).is_less(0.01)
	_tick(fsm, bb, ctx.catch_delay + 0.1)
	assert_str(fsm.current_name()).is_equal("Patrol")
	assert_int(bb.catch_request).is_equal(0)


func test_chase_without_vision_uses_blobs() -> void:
	var ctx := _ctx()
	var bb := _bb()
	bb.vision_enabled = false
	var fsm := _start(ctx, bb, "Chase")
	bb.blobs = [{"angle": 0.0, "distance": 1.0, "width": 0.3}]
	_tick(fsm, bb, 0.05)
	assert_str(fsm.current_name()).is_equal("Catch")
	assert_int(bb.catch_request).is_equal(-1)


func test_failed_blind_catch_marks_blob_as_dud() -> void:
	var ctx := _ctx()
	var bb := _bb()
	bb.vision_enabled = false
	var fsm := _start(ctx, bb, "Chase")
	bb.blobs = [{"angle": 0.0, "distance": 1.0, "width": 0.3}]
	_tick(fsm, bb, 0.05)
	assert_str(fsm.current_name()).is_equal("Catch")
	# Nobody appended to caught_ids during Catch: the blob was a prop or a wall end.
	_tick(fsm, bb, ctx.catch_delay + 0.1)
	assert_str(fsm.current_name()).is_equal("Patrol")
	assert_int(ctx.duds.size()).is_equal(1)
	assert_float((ctx.duds[0]["pos"] as Vector3).distance_to(Vector3(0, 0, -1))).is_less(0.01)
	_tick(fsm, bb, 1.0)  # the same blob is still in view but ignored: no Chase/Catch loop
	assert_str(fsm.current_name()).is_equal("Patrol")


func test_chase_with_no_target_and_nothing_known_goes_to_patrol() -> void:
	var ctx := _ctx()
	var bb := _bb()
	var fsm := _start(ctx, bb, "Chase")
	_tick(fsm, bb, ctx.lose_timeout + 0.2)
	assert_str(fsm.current_name()).is_equal("Patrol")


func test_two_props_in_range_do_not_loop_blind_catch() -> void:
	var bb := _bb()
	bb.vision_enabled = false
	var fsm := _start(_ctx(), bb, "Patrol")
	bb.blobs = [{"angle": 0.0, "distance": 1.0, "width": 0.3}, {"angle": 1.0, "distance": 1.2, "width": 0.3}]
	_tick(fsm, bb, 8.0)
	assert_bool(int(fsm.transition_counts.get("Chase->Catch", 0)) <= 2).is_true()


func test_two_blobs_do_not_ping_pong_investigate() -> void:
	var bb := _bb()
	var fsm := _start(_ctx(), bb, "Patrol")
	bb.goal_reached = true
	bb.blobs = [{"angle": 0.0, "distance": 4.0, "width": 0.3}, {"angle": 1.0, "distance": 5.0, "width": 0.3}]
	_tick(fsm, bb, 9.0)
	assert_bool(int(fsm.transition_counts.get("Patrol->Investigate", 0)) <= 2).is_true()


func test_blind_catch_requested_inside_margin_only() -> void:
	var ctx := _ctx()
	var bb := _bb()
	bb.vision_enabled = false
	var fsm := _start(ctx, bb, "Chase")
	bb.blobs = [{"angle": 0.0, "distance": 1.6, "width": 0.3}]  # blob in chase range, outside 1.2
	_tick(fsm, bb, 0.1)
	assert_str(fsm.current_name()).is_equal("Chase")
	assert_int(bb.catch_request).is_equal(0)
	bb.blobs = [{"angle": 0.0, "distance": 1.1, "width": 0.3}]  # < tag_range - catch_margin = 1.2
	_tick(fsm, bb, 0.05)
	assert_str(fsm.current_name()).is_equal("Catch")
	assert_int(bb.catch_request).is_equal(-1)


func test_dud_expires_after_dud_ttl_not_investigate_cooldown() -> void:
	var ctx := _ctx()
	var bb := _bb()
	bb.vision_enabled = false
	var fsm := _start(ctx, bb, "Chase")
	bb.blobs = [{"angle": 0.0, "distance": 1.0, "width": 0.3}]
	_tick(fsm, bb, 0.05)
	_tick(fsm, bb, ctx.catch_delay + 0.1)  # blind catch caught nobody: dud
	assert_int(ctx.duds.size()).is_equal(1)
	assert_dict(SeekerStates.interesting_blob(bb, ctx, false)).is_empty()
	bb.time += ctx.dud_ttl  # time passes (no FSM ticks, so no new Chase on the same blob)
	assert_float(ctx.dud_ttl).is_less(ctx.investigate_cooldown)
	assert_dict(SeekerStates.interesting_blob(bb, ctx, false)).is_not_empty()
