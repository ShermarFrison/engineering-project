## Text HUD: phase, timers, per-robot FSM state, catches.
class_name HUD
extends CanvasLayer

var _gm: GameManager
@onready var _text: Label = $Text


func bind(gm: GameManager) -> void:
	_gm = gm


func _process(_dt: float) -> void:
	if _gm == null or _gm.config == null:
		_text.text = "no round"
		return
	var lines: PackedStringArray = []
	lines.append("phase: %s   t=%.1f s   seek=%.1f / %.0f s" % [_gm.phase, _gm.time, _gm.seek_time(), _gm.config.round_time])
	lines.append("seed %d  variant %s  hiders %s" % [_gm.config.seed, _gm.config.variant_name, _gm.config.hider_strategy])
	if _gm.seeker:
		lines.append("Seeker: %s  (sensors %s)" % [_gm.seeker.bb.state_name, _gm.config.sensors_label("seeker")])
	var caught: int = 0
	for i: int in _gm.hiders.size():
		var h: RobotBrain = _gm.hiders[i]
		if h.caught:
			caught += 1
		lines.append("Hider%d: %s" % [i, h.bb.state_name])
	lines.append("caught %d / %d" % [caught, _gm.hiders.size()])
	lines.append("[F1] overlay  [R] restart  [Tab] follow next  [0] free cam  RMB drag orbit, wheel zoom")
	_text.text = "\n".join(lines)
