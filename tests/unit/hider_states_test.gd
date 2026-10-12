extends GdUnitTestSuite

const BOUNDS := Rect2(-10, -10, 20, 20)
const WALL := Rect2(-0.15, -10, 0.3, 20)


func _los(a: Vector3, b: Vector3) -> bool:
	return signf(a.x) == signf(b.x)


func _ctx(strategy: String = "static") -> HiderStates.Context:
	var ctx := HiderStates.Context.new()
	ctx.grid = VisibilityGrid.new()
	ctx.seeker_spawn = Vector3(-7, 0, 0)
	ctx.grid.build(BOUNDS, [WALL], ctx.seeker_spawn, _los)
	ctx.strategy = HiderStrategy.create(strategy)
	return ctx


func _bb(pos: Vector3 = Vector3(5, 0, 5)) -> Blackboard:
	var bb := Blackboard.new()
	bb.role = "hider"
	bb.position = pos
	bb.vision_enabled = true
	bb.lidar_enabled = true
	return bb


func _seeker_seen(bb: Blackboard, pos: Vector3) -> void:
	bb.seen_robots = [{"id": 1, "role": "seeker", "position": pos, "bearing": bb.world_to_local_angle(pos),
		"distance": bb.position.distance_to(pos), "time": bb.time}]
	bb.seeker_last_seen_pos = pos
	bb.seeker_last_seen_time = bb.time


func _start(ctx: HiderStates.Context, bb: Blackboard, state: String) -> FSM:
	var fsm := HiderStates.build(ctx)
	fsm.start(state, bb)
	return fsm


func _tick(fsm: FSM, bb: Blackboard, seconds: float) -> void:
	for i: int in int(round(seconds * 60.0)):
		bb.time += 1.0 / 60.0
		fsm.update(bb, 1.0 / 60.0)


func test_pickspot_sets_hidden_goal_and_goes_to_gotospot() -> void:
	var ctx := _ctx()
	var bb := _bb()
	var fsm := _start(ctx, bb, "PickSpot")
	_tick(fsm, bb, 0.05)
	assert_str(fsm.current_name()).is_equal("GoToSpot")
	assert_bool(bb.goal_active).is_true()
	assert_float(bb.goal_pos.x).is_greater(0.0)  # hidden side of the wall
	assert_float(bb.goal_pos.distance_to(ctx.seeker_spawn)).is_greater_equal(ctx.min_hide_dist)


func test_gotospot_to_hide_on_reached() -> void:
	var bb := _bb()
	var fsm := _start(_ctx(), bb, "PickSpot")
	_tick(fsm, bb, 0.05)
	bb.goal_reached = true
	_tick(fsm, bb, 0.05)
	assert_str(fsm.current_name()).is_equal("Hide")
	assert_bool(bb.goal_active).is_false()


func test_gotospot_to_flee_when_seeker_seen() -> void:
	var bb := _bb()
	var fsm := _start(_ctx(), bb, "PickSpot")
	_tick(fsm, bb, 0.05)
	_seeker_seen(bb, Vector3(3, 0, 5))
	_tick(fsm, bb, 0.05)
	assert_str(fsm.current_name()).is_equal("Flee")


func test_gotospot_to_pickspot_when_stuck() -> void:
	var bb := _bb()
	var fsm := _start(_ctx(), bb, "PickSpot")
	_tick(fsm, bb, 0.05)
	bb.stuck = true
	_tick(fsm, bb, 1.0 / 60.0)  # one tick: GoToSpot -> PickSpot
	assert_str(fsm.current_name()).is_equal("PickSpot")
	bb.stuck = false  # Motion clears the flag once a new goal arrives
	_tick(fsm, bb, 1.0 / 60.0)  # PickSpot -> GoToSpot
	assert_str(fsm.current_name()).is_equal("GoToSpot")
	assert_int(fsm.transition_counts.get("GoToSpot->PickSpot", 0)).is_equal(1)


func test_hide_to_flee_when_seeker_seen_and_flee_goal_is_farther() -> void:
	var bb := _bb(Vector3(3, 0, 0))
	var fsm := _start(_ctx(), bb, "Hide")
	var seeker := Vector3(1, 0, 0)
	_seeker_seen(bb, seeker)
	_tick(fsm, bb, 0.05)
	assert_str(fsm.current_name()).is_equal("Flee")
	assert_bool(bb.goal_active).is_true()
	assert_float(bb.goal_pos.distance_to(seeker)).is_greater(bb.position.distance_to(seeker))


func test_hide_to_relocate_on_approaching_blob() -> void:
	var ctx := _ctx()
	var bb := _bb()
	var fsm := _start(ctx, bb, "Hide")
	# first sample (far), then a >= 0.5 m drop within the window
	bb.blobs = [{"angle": 0.0, "distance": ctx.threat_blob_dist, "width": 0.3}]
	_tick(fsm, bb, 0.05)
	bb.blobs = [{"angle": 0.0, "distance": ctx.threat_blob_dist - 0.8, "width": 0.3}]
	_tick(fsm, bb, 0.05)
	assert_str(fsm.current_name()).is_equal("GoToSpot")
	assert_int(fsm.transition_counts.get("Hide->Relocate", 0)).is_equal(1)
	assert_int(fsm.transition_counts.get("Relocate->GoToSpot", 0)).is_equal(1)


func test_stationary_blob_does_not_trigger_relocate() -> void:
	var ctx := _ctx("static")
	var bb := _bb()
	var fsm := _start(ctx, bb, "Hide")
	bb.blobs = [{"angle": 0.0, "distance": 2.0, "width": 0.3}]
	_tick(fsm, bb, 3.0)
	assert_int(fsm.transition_counts.get("Hide->Relocate", 0)).is_equal(0)


func test_flickering_stationary_blob_does_not_trigger_relocate() -> void:
	var ctx := _ctx("static")
	var bb := _bb()
	var fsm := _start(ctx, bb, "Hide")
	for i: int in 180:
		if i % 30 == 29:
			bb.blobs = []
		else:
			bb.blobs = [{"angle": 0.0, "distance": 2.0, "width": 0.3}]
		_tick(fsm, bb, 1.0 / 60.0)
	assert_int(fsm.transition_counts.get("Hide->Relocate", 0)).is_equal(0)


func test_blob_reappearing_stationary_does_not_trigger_relocate() -> void:
	var ctx := _ctx("static")
	var bb := _bb()
	var fsm := _start(ctx, bb, "Hide")
	_tick(fsm, bb, 2.0)
	bb.blobs = [{"angle": 0.0, "distance": 2.0, "width": 0.3}]
	_tick(fsm, bb, 2.0)
	assert_int(fsm.transition_counts.get("Hide->Relocate", 0)).is_equal(0)


func test_approaching_blob_triggers_relocate() -> void:
	var ctx := _ctx("static")
	var bb := _bb()
	var fsm := _start(ctx, bb, "Hide")
	for i: int in 120:
		bb.blobs = [{"angle": 0.0, "distance": lerpf(4.0, 1.5, float(i) / 119.0), "width": 0.3}]
		_tick(fsm, bb, 1.0 / 60.0)
	assert_int(fsm.transition_counts.get("Hide->Relocate", 0)).is_greater_equal(1)


func test_static_hiders_differ_with_different_seeds() -> void:
	var ctx := _ctx("static")
	var picks: Array[Vector3] = []
	for seed_value: int in range(1, 6):
		var bb := _bb()
		bb.rng.seed = seed_value
		var p: Vector3 = ctx.strategy.pick_spot(bb, ctx)
		if not picks.has(p):
			picks.append(p)
	assert_int(picks.size()).is_greater_equal(2)


func test_relocate_leaves_current_spot() -> void:
	var ctx := _ctx("static")
	var bb := _bb()
	bb.rng.seed = 7
	bb.position = ctx.strategy.pick_spot(bb, ctx)
	var fsm := _start(ctx, bb, "Hide")
	fsm.force("Relocate", bb)
	assert_float(bb.goal_pos.distance_to(bb.position)).is_greater(1.5)


func test_flee_to_pickspot_after_seeker_unseen_timeout() -> void:
	var ctx := _ctx()
	var bb := _bb(Vector3(3, 0, 0))
	_seeker_seen(bb, Vector3(1, 0, 0))
	var fsm := _start(ctx, bb, "Flee")
	bb.seen_robots = []
	_tick(fsm, bb, ctx.flee_clear_timeout - 0.5)
	assert_str(fsm.current_name()).is_equal("Flee")
	_tick(fsm, bb, 1.0)
	assert_str(fsm.current_name()).is_equal("GoToSpot")
	assert_int(fsm.transition_counts.get("Flee->PickSpot", 0)).is_equal(1)


func test_flee_refreshes_goal_while_seeker_still_seen() -> void:
	var ctx := _ctx()
	var bb := _bb(Vector3(3, 0, 0))
	var threat := Vector3(1, 0, 0)
	_seeker_seen(bb, threat)
	var fsm := _start(ctx, bb, "Flee")
	var first: Vector3 = bb.goal_pos
	# The hider reached its flee spot and the seeker is still in view: after
	# flee_refresh seconds Flee must pick a new spot, farther from the threat.
	bb.position = first
	_seeker_seen(bb, threat)
	_tick(fsm, bb, ctx.flee_refresh + 0.1)
	assert_str(fsm.current_name()).is_equal("Flee")
	assert_float(bb.goal_pos.distance_to(first)).is_greater(1.0)
	assert_float(bb.goal_pos.distance_to(threat)).is_greater(first.distance_to(threat))


func test_static_strategy_never_relocates_on_timer() -> void:
	var ctx := _ctx("static")
	var bb := _bb()
	var fsm := _start(ctx, bb, "Hide")
	_tick(fsm, bb, 60.0)
	assert_str(fsm.current_name()).is_equal("Hide")


func test_dynamic_strategy_relocates_after_period() -> void:
	var ctx := _ctx("dynamic")
	var bb := _bb()
	var fsm := _start(ctx, bb, "Hide")
	_tick(fsm, bb, 21.0)
	assert_int(fsm.transition_counts.get("Hide->Relocate", 0)).is_greater_equal(1)


func test_dynamic_pick_spot_avoids_current_position() -> void:
	var ctx := _ctx("dynamic")
	var bb := _bb()
	var first: Vector3 = ctx.strategy.pick_spot(bb, ctx)
	bb.position = first
	var second: Vector3 = ctx.strategy.pick_spot(bb, ctx)
	assert_float(second.distance_to(first)).is_greater(1.5)


func test_strategy_factory_kinds() -> void:
	assert_str(HiderStrategy.create("static").kind).is_equal("static")
	assert_str(HiderStrategy.create("dynamic").kind).is_equal("dynamic")


func _xz_dist(a: Vector3, b: Vector3) -> float:
	return Vector2(a.x, a.z).distance_to(Vector2(b.x, b.z))


func test_choose_spot_skips_spots_near_a_known_hider() -> void:
	var ctx := _ctx("static")
	var top: Array[Vector3] = ctx.grid.pick_hiding_spots(ctx.seeker_spawn, ctx.min_hide_dist, HiderStrategy.TOP_K)
	var other: Vector3 = top[0]
	var far_left: int = 0
	for c: Vector3 in top:
		if _xz_dist(c, other) >= ctx.hider_spacing:
			far_left += 1
	assert_int(far_left).is_greater(0)  # the filter leaves something to pick from
	for seed_value: int in range(1, 21):
		var bb := _bb()
		bb.rng.seed = seed_value
		bb.last_known[123] = {"position": other, "time": 0.0, "role": "hider"}
		var p: Vector3 = HiderStrategy.choose_spot(bb, ctx, bb.position, 0.0)
		assert_float(_xz_dist(p, other)).is_greater_equal(ctx.hider_spacing)


func test_choose_spot_ignores_known_seeker_for_spacing() -> void:
	var ctx := _ctx("static")
	var top: Array[Vector3] = ctx.grid.pick_hiding_spots(ctx.seeker_spawn, ctx.min_hide_dist, HiderStrategy.TOP_K)
	var hits: int = 0
	for seed_value: int in range(1, 41):
		var bb := _bb()
		bb.rng.seed = seed_value
		bb.last_known[7] = {"position": top[0], "time": 0.0, "role": "seeker"}
		if HiderStrategy.choose_spot(bb, ctx, bb.position, 0.0) == top[0]:
			hits += 1
	assert_int(hits).is_greater(0)


func test_choose_spot_falls_back_to_full_list_when_all_crowded() -> void:
	var ctx := _ctx("static")
	var top: Array[Vector3] = ctx.grid.pick_hiding_spots(ctx.seeker_spawn, ctx.min_hide_dist, HiderStrategy.TOP_K)
	var bb := _bb()
	bb.rng.seed = 3
	for i: int in top.size():
		bb.last_known[1000 + i] = {"position": top[i], "time": 0.0, "role": "hider"}
	assert_bool(top.has(HiderStrategy.choose_spot(bb, ctx, bb.position, 0.0))).is_true()
