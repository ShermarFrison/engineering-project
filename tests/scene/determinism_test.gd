extends PhysicsTestSuite

## Rounds run in the same scene back to back (as BatchRunner does), with no warm-up round:
## GameManager.start_round only latches the config and does the setup on its own next
## physics tick, so a round does not depend on where (or in which frame phase) it was started.

const ARENA := preload("res://scenes/arena/arena.tscn")


## Starts one round from its own _process callback, once (a different call site than
## call_deferred from a physics tick or a direct call from the test coroutine).
class ProcessStarter extends Node:
	var gm: GameManager
	var cfg: RoundConfig

	func _process(_dt: float) -> void:
		if cfg != null:
			gm.start_round(cfg)
			cfg = null


func _make_game() -> GameManager:
	var root := Node3D.new()
	var arena: Arena = ARENA.instantiate()
	arena.name = "Arena"
	root.add_child(arena)
	var gm := GameManager.new()
	gm.arena_path = NodePath("../Arena")
	root.add_child(gm)
	var runner := scene_runner(auto_free(root))  # the runner itself is auto-freed by gdUnit
	assert_object(runner.scene()).is_same(root)
	return gm


func _config(round_seed: int) -> RoundConfig:
	var cfg := RoundConfig.new()
	cfg.seed = round_seed
	cfg.hider_count = 2
	cfg.head_start = 1.0
	cfg.round_time = 6.0
	cfg.prop_count = 8
	return cfg


## Waits for the round already started on gm to finish and returns its CSV row.
func _finish_round(gm: GameManager, cfg: RoundConfig) -> PackedStringArray:
	var budget: int = int((cfg.head_start + cfg.round_time + 5.0) * 60.0)
	var done: bool = await ticks_until(func() -> bool: return gm.phase == "finished" and gm.config == cfg, budget)
	assert_bool(done).append_failure_message("seed %d: phase %s after %d ticks" % [cfg.seed, gm.phase, budget]).is_true()
	if not done:
		return PackedStringArray()
	await ticks(2)  # let queue_free of the previous round settle before the next
	return ResultsCsv.row(gm.results())


func _run_round(gm: GameManager, round_seed: int) -> PackedStringArray:
	var cfg := _config(round_seed)
	gm.start_round(cfg)
	return await _finish_round(gm, cfg)


## The row without its `seed` column, which differs by construction between seeds.
func _without_seed(row: PackedStringArray) -> Array:
	var a: Array = Array(row)
	a.remove_at(ResultsCsv.header().find("seed"))
	return a


func test_same_seed_same_results() -> void:
	var gm := _make_game()
	var a: PackedStringArray = await _run_round(gm, 21)
	var b: PackedStringArray = await _run_round(gm, 21)
	assert_int(a.size()).is_equal(ResultsCsv.header().size())
	assert_array(Array(a)).is_equal(Array(b))


func test_different_seed_different_results() -> void:
	var gm := _make_game()
	var a: PackedStringArray = await _run_round(gm, 21)
	var b: PackedStringArray = await _run_round(gm, 22)
	assert_int(a.size()).is_equal(ResultsCsv.header().size())
	assert_int(b.size()).is_equal(ResultsCsv.header().size())
	assert_array(_without_seed(a)).is_not_equal(_without_seed(b))


func test_same_seed_same_rows_regardless_of_start_call_site() -> void:
	var gm := _make_game()
	# (a) deferred from the test coroutine, as BatchRunner does from round_finished.
	var cfg_a := _config(7)
	gm.call_deferred("start_round", cfg_a)
	var a: PackedStringArray = await _finish_round(gm, cfg_a)
	# (b) from a node's _process callback, as an input handler (main.gd, key R) does.
	var starter := ProcessStarter.new()
	starter.gm = gm
	var cfg_b := _config(7)
	starter.cfg = cfg_b
	gm.add_child(starter)
	var b: PackedStringArray = await _finish_round(gm, cfg_b)
	assert_int(a.size()).is_equal(ResultsCsv.header().size())
	assert_array(Array(a)).is_equal(Array(b))
