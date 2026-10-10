extends PhysicsTestSuite

## Both rounds run in the same scene back to back (as BatchRunner does). The first
## round played in a process differs from every later same-seed round (cause open,
## see NOTES/godot-headless-gotchas.md), so each test plays one discarded warm-up round.

const ARENA := preload("res://scenes/arena/arena.tscn")


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


func _run_round(gm: GameManager, round_seed: int) -> PackedStringArray:
	var cfg := RoundConfig.new()
	cfg.seed = round_seed
	cfg.hider_count = 2
	cfg.head_start = 1.0
	cfg.round_time = 6.0
	cfg.prop_count = 8
	gm.start_round(cfg)
	var budget: int = int((cfg.head_start + cfg.round_time + 5.0) * 60.0)
	var done: bool = await ticks_until(func() -> bool: return gm.phase == "finished", budget)
	assert_bool(done).append_failure_message("seed %d: phase %s after %d ticks" % [round_seed, gm.phase, budget]).is_true()
	if not done:
		return PackedStringArray()
	await ticks(2)  # let queue_free of the previous round settle before the next
	return ResultsCsv.row(gm.results())


## The row without its `seed` column, which differs by construction between seeds.
func _without_seed(row: PackedStringArray) -> Array:
	var a: Array = Array(row)
	a.remove_at(ResultsCsv.header().find("seed"))
	return a


func test_same_seed_same_results() -> void:
	var gm := _make_game()
	await _run_round(gm, 0)  # discarded warm-up round (see above)
	var a: PackedStringArray = await _run_round(gm, 21)
	var b: PackedStringArray = await _run_round(gm, 21)
	assert_int(a.size()).is_equal(ResultsCsv.header().size())
	assert_array(Array(a)).is_equal(Array(b))


func test_different_seed_different_results() -> void:
	var gm := _make_game()
	await _run_round(gm, 0)  # discarded warm-up round (see above)
	var a: PackedStringArray = await _run_round(gm, 21)
	var b: PackedStringArray = await _run_round(gm, 22)
	assert_int(a.size()).is_equal(ResultsCsv.header().size())
	assert_int(b.size()).is_equal(ResultsCsv.header().size())
	assert_array(_without_seed(a)).is_not_equal(_without_seed(b))
