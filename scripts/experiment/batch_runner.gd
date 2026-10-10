## Headless experiment driver. Usage (from project root):
##   ./Godot_v4.7.2-stable_linux.x86_64 --headless --path . res://scenes/batch.tscn -- \
##       --config res://experiments/ablation.tres --rounds 50 --out experiments/ablation.csv
class_name BatchRunner
extends Node

@onready var gm: GameManager = $GameManager

var _variants: Array[RoundConfig] = []
var _rounds: int = 20
var _out_path: String = "experiments/results.csv"
var _log_transitions: bool = false
var _seed_offset: int = 0
var _time_scale: float = 8.0

var _vi: int = 0
var _si: int = 0
var _csv: FileAccess
var _tcsv: FileAccess
var _summary: Dictionary = {}  # variant -> {rounds, caught, total, duration_sum}
var _pending: bool = false  # a round was started and has neither finished nor been skipped
var _warming_up: bool = true  # the first round of a process is played and discarded


func _parse_args() -> bool:
	var args: PackedStringArray = OS.get_cmdline_user_args()
	var config_path: String = ""
	var i: int = 0
	while i < args.size():
		var a: String = args[i]
		var has_val: bool = i + 1 < args.size()
		match a:
			"--config":
				if not has_val:
					return false
				config_path = args[i + 1]
				i += 1
			"--rounds":
				if not has_val:
					return false
				_rounds = int(args[i + 1])
				i += 1
			"--out":
				if not has_val:
					return false
				_out_path = args[i + 1]
				i += 1
			"--time-scale":
				if not has_val:
					return false
				_time_scale = float(args[i + 1])
				i += 1
			"--seed-offset":
				if not has_val:
					return false
				_seed_offset = int(args[i + 1])
				i += 1
			"--log-transitions":
				_log_transitions = true
			_:
				push_error("unknown arg %s" % a)
				return false
		i += 1
	if config_path == "":
		push_error("--config is required")
		return false
	var cfg := load(config_path) as ExperimentConfig
	if cfg == null:
		push_error("cannot load %s as ExperimentConfig" % config_path)
		return false
	_variants = cfg.variants
	if not args.has("--rounds"):
		_rounds = cfg.rounds_per_variant
	if _rounds < 1:
		push_error("--rounds must be at least 1 (got %d)" % _rounds)
		return false
	return not _variants.is_empty()


func _ready() -> void:
	if not _parse_args():
		get_tree().quit(2)
		return
	Engine.time_scale = _time_scale
	Engine.max_physics_steps_per_frame = maxi(8, int(ceil(_time_scale)) * 2)
	var abs_out: String = _out_path
	if _out_path.begins_with("res://") or _out_path.begins_with("user://"):
		abs_out = ProjectSettings.globalize_path(_out_path)
	elif not _out_path.is_absolute_path():
		abs_out = ProjectSettings.globalize_path("res://").path_join(_out_path)
	_csv = FileAccess.open(abs_out, FileAccess.WRITE)
	if _csv == null:
		push_error("cannot open %s" % abs_out)
		get_tree().quit(2)
		return
	_csv.store_csv_line(ResultsCsv.header())
	if _log_transitions:
		var tpath: String = abs_out.get_basename() + "_transitions.csv"
		_tcsv = FileAccess.open(tpath, FileAccess.WRITE)
		if _tcsv == null:
			push_error("cannot open %s" % tpath)
			get_tree().quit(2)
			return
		_tcsv.store_csv_line(ResultsCsv.transitions_header())
	gm.round_finished.connect(_on_round_finished)
	_start_next()


func _start_next() -> void:
	if _vi >= _variants.size():
		_finish()
		return
	var cfg: RoundConfig = _variants[_vi].duplicate()
	cfg.seed = _seed_offset + _si
	if _warming_up:
		print("[batch] warm-up round (discarded): variant=%s seed=%d" % [cfg.variant_name, cfg.seed])
	else:
		print("[batch] variant=%s seed=%d (%d/%d)" % [cfg.variant_name, cfg.seed, _si + 1, _rounds])
	_pending = true
	gm.start_round(cfg)


## GameManager drops back to "idle" without emitting round_finished when its setup
## times out (navmesh never ready within 600 ticks). Skip that round instead of waiting forever.
func _physics_process(_dt: float) -> void:
	if _pending and gm.phase == "idle":
		_on_round_finished({})


## Advances the batch; driven by GameManager.round_finished (or the idle watchdog above).
func _on_round_finished(res: Dictionary) -> void:
	_pending = false
	if _warming_up:
		# The first round of a process is not reproducible by later same-seed rounds; drop it.
		_warming_up = false
		call_deferred("_start_next")
		return
	if res.is_empty():
		push_error("[batch] variant=%s seed=%d produced no results; round skipped" % [_variants[_vi].variant_name, _seed_offset + _si])
	else:
		_record(res)
	_si += 1
	if _si >= _rounds:
		_si = 0
		_vi += 1
	call_deferred("_start_next")


func _record(res: Dictionary) -> void:
	_csv.store_csv_line(ResultsCsv.row(res))
	_csv.flush()
	if _tcsv:
		for r: PackedStringArray in ResultsCsv.transitions_rows(res):
			_tcsv.store_csv_line(r)
		_tcsv.flush()
	var v: String = res["variant"]
	if not _summary.has(v):
		_summary[v] = {"rounds": 0, "caught": 0, "total": 0, "duration_sum": 0.0}
	_summary[v]["rounds"] += 1
	_summary[v]["caught"] += int(res["hiders_caught"])
	_summary[v]["total"] += int(res["hiders_total"])
	_summary[v]["duration_sum"] += float(res["duration"])


func _finish() -> void:
	_csv.close()
	if _tcsv:
		_tcsv.close()
	print("\n%-24s %6s %10s %10s" % ["variant", "rounds", "catch_rate", "mean_dur"])
	for v: String in _summary:
		var s: Dictionary = _summary[v]
		print("%-24s %6d %10.3f %10.1f" % [v, s["rounds"], float(s["caught"]) / maxi(1, s["total"]), s["duration_sum"] / maxi(1, s["rounds"])])
	print("[batch] wrote %s" % _out_path)
	get_tree().quit(0)
