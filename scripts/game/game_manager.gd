## Owns the round: RNG, props, navmesh, robots, phases, catch detection, results.
class_name GameManager
extends Node

signal round_started(config: RoundConfig)
signal round_finished(results: Dictionary)

const ROBOT := preload("res://scenes/robots/robot.tscn")
const SETUP_MAX_TICKS: int = 600  # 10 s at 60 Hz; the navmesh is normally ready in ~3 ticks
## Setup ticks before the first navmesh readiness check. A fixed count, so the tick on which
## the robots spawn does not depend on sync timing as long as the navmesh is ready by then
## (it normally is by tick ~3); if it is not, the spawn tick follows the sync again.
const SETUP_MIN_TICKS: int = 10

@export var arena_path: NodePath

var arena: Arena
var phase: String = "idle"
var time: float = 0.0
var config: RoundConfig
var rng: RandomNumberGenerator = RandomNumberGenerator.new()
var grid: VisibilityGrid
var seeker: RobotBrain
var hiders: Array[RobotBrain] = []

var _spawner: PropSpawner = PropSpawner.new()
var _catch_times: Array[float] = []
var _results: Dictionary = {}
var _setup_ticks: int = 0
var _setup_pending: bool = false  # start_round latched a config; setup runs on the next physics tick


func _ready() -> void:
	arena = get_node(arena_path) as Arena
	add_child(_spawner)


func seek_time() -> float:
	return maxf(0.0, time - config.head_start) if config else 0.0


func los_clear(a: Vector3, b: Vector3, mask: int) -> bool:
	var q := PhysicsRayQueryParameters3D.create(a + Vector3(0, 0.3, 0), b + Vector3(0, 0.3, 0), mask)
	return arena.get_world_3d().direct_space_state.intersect_ray(q).is_empty()


func _los_walls(a: Vector3, b: Vector3) -> bool:
	return los_clear(a, b, 1)


## Latches the config only. The setup (clear, props, navmesh bake) runs on this node's next
## physics tick, so a round is the same whether it was started from a physics callback,
## call_deferred or a _process/input callback (see NOTES/godot-headless-gotchas.md).
## phase is "setup" at once, so watchdogs polling for "idle" see the round as pending.
func start_round(cfg: RoundConfig) -> void:
	config = cfg
	_results = {}
	_setup_pending = true
	_setup_ticks = 0
	phase = "setup"


func _begin_setup() -> void:
	rng = RandomNumberGenerator.new()
	rng.seed = config.seed
	time = 0.0
	_catch_times.clear()
	# Clear previous round.
	for r: RobotBrain in hiders:
		r.body.queue_free()
	if seeker:
		seeker.body.queue_free()
	hiders.clear()
	seeker = null
	_spawner.clear(arena.props_root)
	# Props (seeded), keep spawns clear. Typed list built with append_array: `[a] + typed`
	# is an untyped Array and fails at runtime when assigned to Array[Vector3].
	var keep: Array[Vector3] = [arena.seeker_spawn_point()]
	keep.append_array(arena.hider_spawn_points())
	var layout := PropLayout.generate(rng, config.prop_count, arena.bounds(), arena.wall_rects(), keep, PropSpawner.SIZES)
	_spawner.spawn(layout, arena.props_root)
	arena.nav.bake(arena.static_root)
	_setup_ticks = 0


## Instantiates one robot under arena.robots_root and wires its brain (both roles).
func _spawn_one(role: String, pos: Vector3, index: int) -> RobotBrain:
	var body: DriveBody = ROBOT.instantiate()
	body.name = "Seeker" if role == "seeker" else "Hider%d" % index
	body.role = role
	body.position = pos + Vector3(0, 0.3, 0)
	if role == "hider":
		body.rotation.y = rng.randf_range(-PI, PI)
	arena.robots_root.add_child(body)
	var brain: RobotBrain = body.get_node("Brain")
	brain.setup(role, rng, arena.nav, grid, _los_walls, config, arena.seeker_spawn_point())
	return brain


func _spawn_robots() -> void:
	seeker = _spawn_one("seeker", arena.seeker_spawn_point(), 0)
	# Hiders at a seeded random subset of spawn markers.
	var spawns: Array[Vector3] = arena.hider_spawn_points()
	for i: int in range(spawns.size() - 1, 0, -1):  # Fisher-Yates with our RNG
		var j: int = rng.randi_range(0, i)
		var tmp: Vector3 = spawns[i]
		spawns[i] = spawns[j]
		spawns[j] = tmp
	for i: int in mini(config.hider_count, spawns.size()):
		hiders.append(_spawn_one("hider", spawns[i], i))
		_catch_times.append(-1.0)


func _physics_process(dt: float) -> void:
	match phase:
		"setup":
			if _setup_pending:
				_setup_pending = false
				_begin_setup()
				return
			_setup_ticks += 1
			if _setup_ticks < SETUP_MIN_TICKS:
				return
			if not arena.nav.is_ready():
				if _setup_ticks >= SETUP_MAX_TICKS:
					push_error("nav not ready")
					phase = "idle"  # no signal; results() stays {} and BatchRunner skips the round
				return
			grid = VisibilityGrid.new()
			grid.build(arena.bounds(), arena.wall_rects(), arena.seeker_spawn_point(), _los_walls)
			_spawn_robots()
			seeker.active = true
			for h: RobotBrain in hiders:
				h.active = true
			phase = "head_start"
			round_started.emit(config)
		"head_start":
			time += dt
			if time >= config.head_start:
				phase = "seeking"
				seeker.bb.seeking_allowed = true
		"seeking":
			time += dt
			_check_catches()
			var all_caught: bool = true
			for h: RobotBrain in hiders:
				if not h.caught:
					all_caught = false
			if all_caught or seek_time() >= config.round_time:
				_finish()


func _check_catches() -> void:
	var req: int = seeker.bb.catch_request
	if req == 0:
		return
	var sp: Vector3 = seeker.body.global_position
	for i: int in hiders.size():
		var h: RobotBrain = hiders[i]
		if h.caught:
			continue
		var id: int = h.body.get_instance_id()
		if req != -1 and req != id:
			continue
		var hp: Vector3 = h.body.global_position
		var d: float = Vector2(sp.x, sp.z).distance_to(Vector2(hp.x, hp.z))
		if CatchRule.is_catch(d, config.tag_range, seeker.bb.state_name, los_clear(sp, hp, 3)):
			h.set_caught()
			seeker.bb.caught_ids.append(id)
			# Remember where it was caught, so a seeker without vision ignores its blob from now on.
			seeker.bb.last_known[id] = {"position": hp, "time": seeker.bb.time, "role": "hider"}
			_catch_times[i] = seek_time()


func _finish() -> void:
	phase = "finished"
	seeker.active = false
	for h: RobotBrain in hiders:
		h.active = false
		h.body.set_command(0.0, 0.0)
	seeker.body.set_command(0.0, 0.0)
	var hider_rows: Array[Dictionary] = []
	var caught: int = 0
	for i: int in hiders.size():
		var s: Dictionary = hiders[i].stats()
		s["index"] = i
		s["caught"] = hiders[i].caught
		s["catch_time"] = _catch_times[i]
		s.erase("state")
		if hiders[i].caught:
			caught += 1
		hider_rows.append(s)
	var seeker_stats: Dictionary = seeker.stats()
	seeker_stats.erase("state")
	_results = {
		"variant": config.variant_name,
		"seed": config.seed,
		"hider_strategy": config.hider_strategy,
		"seeker_sensors": config.sensors_label("seeker"),
		"hider_sensors": config.sensors_label("hider"),
		"hiders_total": hiders.size(),
		"hiders_caught": caught,
		"duration": seek_time(),
		"seeker": seeker_stats,
		"hiders": hider_rows,
	}
	round_finished.emit(_results)


func results() -> Dictionary:
	return _results
