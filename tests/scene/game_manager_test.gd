extends PhysicsTestSuite

const ARENA := preload("res://scenes/arena/arena.tscn")
const CRATE := preload("res://scenes/props/crate.tscn")


func _game() -> GameManager:
	var root := Node3D.new()
	var arena: Arena = ARENA.instantiate()
	arena.name = "Arena"
	root.add_child(arena)
	var gm := GameManager.new()
	gm.name = "GameManager"
	gm.arena_path = NodePath("../Arena")
	root.add_child(gm)
	var runner := scene_runner(auto_free(root))  # the runner itself is auto-freed by gdUnit
	assert_object(runner.scene()).is_same(root)
	return gm


func _short_config(seed: int = 3) -> RoundConfig:
	var c := RoundConfig.new()
	c.seed = seed
	c.hider_count = 1
	c.head_start = 1.0
	c.round_time = 4.0
	c.prop_count = 6
	return c


## Physics-tick budget for a whole round: setup + head start + seek phase + 5 s slack.
func _budget(cfg: RoundConfig) -> int:
	return int((cfg.head_start + cfg.round_time + 5.0) * 60.0)


func _run_to_end(gm: GameManager, cfg: RoundConfig) -> bool:
	return await ticks_until(func() -> bool: return gm.phase == "finished", _budget(cfg))


func _wait_setup(gm: GameManager) -> bool:
	return await ticks_until(func() -> bool: return gm.phase != "setup", 600)


func test_round_lifecycle_emits_started_and_finished() -> void:
	var gm := _game()
	var cfg := _short_config()
	var monitored := monitor_signals(gm)
	gm.start_round(cfg)
	assert_bool(await _run_to_end(gm, cfg)).append_failure_message("phase %s" % gm.phase).is_true()
	# Both signals carry one argument; a bare is_emitted(name) only matches zero-arg emissions.
	await assert_signal(monitored).is_emitted("round_started", [any()])
	await assert_signal(monitored).is_emitted("round_finished", [any()])
	var res: Dictionary = gm.results()
	assert_int(res["hiders_total"]).is_equal(1)
	assert_int(res["seed"]).is_equal(3)
	assert_int(res["hiders"].size()).is_equal(1)
	assert_dict(res["seeker"]).contains_keys(["distance", "collisions", "stuck_events", "transitions", "transition_counts"])


func test_round_ends_on_timeout() -> void:
	var gm := _game()
	var cfg := _short_config()
	cfg.seeker_sensors = PackedStringArray([])  # blind seeker cannot catch anyone
	gm.start_round(cfg)
	assert_bool(await _run_to_end(gm, cfg)).append_failure_message("phase %s" % gm.phase).is_true()
	var res: Dictionary = gm.results()
	assert_int(res["hiders_caught"]).is_equal(0)
	assert_bool(res["hiders"][0]["caught"]).is_false()
	assert_float(res["hiders"][0]["catch_time"]).is_equal(-1.0)
	assert_float(res["duration"]).is_equal_approx(cfg.round_time, 0.1)


func test_seeker_catches_adjacent_visible_hider() -> void:
	var gm := _game()
	var cfg := _short_config()
	cfg.head_start = 0.5
	cfg.round_time = 10.0
	cfg.prop_count = 0
	gm.start_round(cfg)
	assert_bool(await _wait_setup(gm)).is_true()
	var hider: RobotBrain = gm.hiders[0]
	hider.active = false  # freeze it: RobotBrain then holds its body at zero command
	var sp: Vector3 = gm.seeker.body.global_position
	hider.body.global_position = sp + gm.seeker.body.forward() * 3.0
	hider.body.rotation.y = gm.seeker.body.rotation.y
	assert_bool(await _run_to_end(gm, cfg)).append_failure_message("phase %s" % gm.phase).is_true()
	var res: Dictionary = gm.results()
	assert_int(res["hiders_caught"]).append_failure_message("seeker transitions %s" % [gm.seeker.fsm.transitions]).is_equal(1)
	assert_float(res["hiders"][0]["catch_time"]).is_between(0.0, 6.0)
	assert_int(res["seeker"]["transition_counts"].get("Chase->Catch", 0)).is_greater_equal(1)


## End to end blind catch: lidar (blobs) and bumper only, hider frozen 2.5 m ahead in the
## open. The seeker must close inside tag range itself (blob distance is to the mast surface).
func test_lidar_only_seeker_catches_frozen_hider_in_open_floor() -> void:
	var gm := _game()
	var cfg := _short_config()
	cfg.head_start = 0.5
	cfg.round_time = 15.0
	cfg.prop_count = 0
	cfg.seeker_sensors = PackedStringArray(["lidar", "bumper"])
	gm.start_round(cfg)
	assert_bool(await _wait_setup(gm)).is_true()
	var hider: RobotBrain = gm.hiders[0]
	hider.active = false
	hider.body.global_position = gm.seeker.body.global_position + gm.seeker.body.forward() * 2.5
	assert_bool(await ticks_until(func() -> bool: return gm.seeker.bb.seeking_allowed, 60)).is_true()
	assert_bool(await _run_to_end(gm, cfg)).append_failure_message("phase %s" % gm.phase).is_true()
	var res: Dictionary = gm.results()
	assert_int(res["hiders_caught"]).append_failure_message("seeker transitions %s" % [gm.seeker.fsm.transitions]).is_equal(1)
	assert_float(res["hiders"][0]["catch_time"]).is_between(0.0, 15.0)


func test_blind_seeker_touching_hider_does_not_catch() -> void:
	var gm := _game()
	var cfg := _short_config()
	cfg.head_start = 0.5
	cfg.round_time = 3.0
	cfg.prop_count = 0
	cfg.seeker_sensors = PackedStringArray(["bumper"])  # no vision, no lidar: no detection at all
	gm.start_round(cfg)
	assert_bool(await _wait_setup(gm)).is_true()
	var hider: RobotBrain = gm.hiders[0]
	hider.active = false
	hider.body.global_position = gm.seeker.body.global_position + gm.seeker.body.forward() * 1.0
	assert_bool(await _run_to_end(gm, cfg)).append_failure_message("phase %s" % gm.phase).is_true()
	assert_int(gm.results()["hiders_caught"]).is_equal(0)


## Seeker and frozen hider 1.0 m apart (inside tag range 1.5) with the seeker forced
## into Chase and a catch requested for that hider. Both brains are inactive, so the
## bodies hold still and only the GameManager's catch rule decides (distance, state,
## mask-3 walls+props LOS ray). With with_crate, a thin crate stands between them.
func _face_off(with_crate: bool) -> GameManager:
	var gm := _game()
	var cfg := _short_config()
	cfg.head_start = 0.5
	cfg.round_time = 3.0
	cfg.prop_count = 0
	gm.start_round(cfg)
	assert_bool(await ticks_until(func() -> bool: return gm.phase == "seeking", 600 + 60)).is_true()
	var seeker: RobotBrain = gm.seeker
	var hider: RobotBrain = gm.hiders[0]
	seeker.active = false
	hider.active = false
	await ticks(30)  # let both bodies decelerate to a stop (rate-limited wheels)
	var sp: Vector3 = seeker.body.global_position
	var fwd: Vector3 = seeker.body.forward()
	hider.body.global_position = sp + fwd * 1.0
	hider.body.rotation.y = seeker.body.rotation.y
	if with_crate:
		# 1.5 m wide, 1 m tall, 0.2 m thick, across the line of sight halfway between
		# them: 0.15 m clear of the seeker's front (sp - 0.25) and the hider's back (sp - 0.75).
		var crate: StaticBody3D = CRATE.instantiate()
		crate.scale = Vector3(1.5, 1.0, 0.2)
		crate.rotation.y = seeker.body.rotation.y
		crate.position = Vector3(sp.x, 0.5, sp.z) + fwd * 0.5
		gm.arena.props_root.add_child(crate)
	seeker.fsm.force("Chase", seeker.bb)
	seeker.bb.catch_request = hider.body.get_instance_id()
	return gm


func test_hider_behind_crate_not_caught() -> void:
	var gm: GameManager = await _face_off(true)
	await ticks(120)
	assert_str(gm.seeker.bb.state_name).is_equal("Chase")
	assert_bool(gm.hiders[0].caught).is_false()
	assert_bool(await _run_to_end(gm, gm.config)).append_failure_message("phase %s" % gm.phase).is_true()
	assert_int(gm.results()["hiders_caught"]).is_equal(0)


func test_same_face_off_without_crate_is_caught() -> void:
	# Positive control for the test above: identical geometry and request, no crate.
	var gm: GameManager = await _face_off(false)
	assert_bool(await ticks_until(func() -> bool: return gm.hiders[0].caught, 10)).is_true()


func test_spawns_configured_hider_count_and_props() -> void:
	var gm := _game()
	var cfg := _short_config()
	cfg.hider_count = 3
	cfg.prop_count = 10
	gm.start_round(cfg)
	assert_bool(await _wait_setup(gm)).is_true()
	assert_int(gm.hiders.size()).is_equal(3)
	assert_int(gm.arena.props_root.get_child_count()).is_equal(10)
	assert_int(gm.arena.robots_root.get_child_count()).is_equal(4)
