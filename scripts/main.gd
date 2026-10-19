## Visual entry point: start a round from the default config, handle hotkeys.
extends Node3D

const DEFAULT_CONFIG := preload("res://experiments/default_round.tres")

@onready var gm: GameManager = $GameManager
@onready var cam: CameraRig = $CameraRig
@onready var hud: HUD = $HUD

var _follow_index: int = -1


func _ready() -> void:
	hud.bind(gm)
	gm.round_finished.connect(func(res: Dictionary) -> void: print(JSON.stringify(res, "  ")))
	gm.round_started.connect(_on_round_started)
	# A fresh seed per launch, so every start gets a new maze; batch runs keep their seeds.
	var cfg: RoundConfig = DEFAULT_CONFIG.duplicate()
	cfg.seed = randi()
	gm.start_round(cfg)


## The previous round's robots are freed on restart: drop any follow target.
func _on_round_started(_config: RoundConfig) -> void:
	_follow_index = -1
	cam.free_camera()


func _unhandled_input(event: InputEvent) -> void:
	if event.is_action_pressed("toggle_overlay"):
		DebugOverlay.visible_globally = not DebugOverlay.visible_globally
	elif event.is_action_pressed("restart_round"):
		var cfg: RoundConfig = DEFAULT_CONFIG.duplicate()
		cfg.seed = gm.config.seed + 1 if gm.config else cfg.seed
		gm.start_round(cfg)
	elif event.is_action_pressed("follow_next"):
		var bodies: Array[Node3D] = []
		if gm.seeker:
			bodies.append(gm.seeker.body)
		for h: RobotBrain in gm.hiders:
			bodies.append(h.body)
		if bodies.is_empty():
			return
		_follow_index = (_follow_index + 1) % bodies.size()
		cam.follow(bodies[_follow_index])
	elif event.is_action_pressed("free_camera"):
		_follow_index = -1
		cam.free_camera()
