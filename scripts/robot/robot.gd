## Ties the layers together: sensors -> blackboard -> FSM -> motion -> drive.
## Lives on the Brain node; its parent is the DriveBody.
class_name RobotBrain
extends Node

var bb: Blackboard = Blackboard.new()
var fsm: FSM
var motion: Motion = Motion.new()
var body: DriveBody
var active: bool = false
var caught: bool = false
var stuck_events: int = 0

var _lidar: Lidar
var _vision: VisionCone
var _ultrasonic: Ultrasonic
var _bumper: Bumper
var _overlay: DebugOverlay
var _sensors: Array[SensorBase] = []
var _was_stuck: bool = false
var _walls: Array[Rect2] = []  # static layout, to drop wall-end blobs


func _ready() -> void:
	body = get_parent() as DriveBody
	_lidar = body.get_node("Sensors/Lidar")
	_vision = body.get_node("Sensors/Vision")
	_ultrasonic = body.get_node("Sensors/Ultrasonic")
	_bumper = body.get_node("Sensors/Bumper")
	_overlay = body.get_node_or_null("DebugOverlay")
	_sensors = [_lidar, _vision, _ultrasonic, _bumper]
	for s: SensorBase in _sensors:
		s.body = body


func _enable_sensors(role: String, config: RoundConfig) -> void:
	_lidar.enabled = config.has_sensor(role, "lidar")
	_vision.enabled = config.has_sensor(role, "vision")
	_ultrasonic.enabled = config.has_sensor(role, "ultrasonic")
	_bumper.enabled = config.has_sensor(role, "bumper")
	for s: SensorBase in _sensors:
		s.noise_std = config.noise_std


func setup(role: String, rng: RandomNumberGenerator, nav: NavService, grid: VisibilityGrid,
		los: Callable, config: RoundConfig, seeker_spawn: Vector3) -> void:
	body.role = role
	bb.role = role
	bb.rng = rng
	_walls = grid.wall_rects
	_enable_sensors(role, config)
	motion.path_provider = nav.find_path
	motion.max_v = body.model.max_linear_speed() * 0.8
	if role == "seeker":
		var ctx := SeekerStates.Context.new()
		ctx.grid = grid
		ctx.los = los
		ctx.tag_range = config.tag_range
		ctx.fov = deg_to_rad(_vision.fov_deg)
		ctx.vision_range = _vision.max_range
		fsm = SeekerStates.build(ctx)
		fsm.start("Wait", bb)
		_set_color(Color(0.9, 0.2, 0.2))
	else:
		var hctx := HiderStates.Context.new()
		hctx.grid = grid
		hctx.seeker_spawn = seeker_spawn
		hctx.strategy = HiderStrategy.create(config.hider_strategy)
		fsm = HiderStates.build(hctx)
		fsm.start("PickSpot", bb)
		_set_color(Color(0.2, 0.6, 0.9))


func setup_goto(nav: NavService, goal: Vector3) -> void:
	bb.role = body.role
	for s: SensorBase in _sensors:
		s.enabled = true
	motion.path_provider = nav.find_path
	motion.max_v = body.model.max_linear_speed() * 0.8
	fsm = FSM.new()
	fsm.add(GoToState.new(goal))
	fsm.start("GoTo", bb)


func set_caught() -> void:
	caught = true
	bb.goal_active = false
	bb.state_name = "Caught"
	body.set_command(0.0, 0.0)
	_set_color(Color(0.4, 0.4, 0.4))


func _set_color(c: Color) -> void:
	var mesh := body.get_node("Chassis") as MeshInstance3D
	var mat := StandardMaterial3D.new()
	mat.albedo_color = c
	mesh.set_surface_override_material(0, mat)


func stats() -> Dictionary:
	return {
		"distance": body.distance_travelled,
		"collisions": body.collision_count,
		"stuck_events": stuck_events,
		"transitions": fsm.transitions.duplicate() if fsm else [],
		"transition_counts": fsm.transition_counts.duplicate() if fsm else {},
		"state": bb.state_name,
	}


func _physics_process(dt: float) -> void:
	if fsm == null:
		return  # not set up: drive and sensor scene tests command the body directly
	if not active:
		# Frozen (by a test) or round over: DriveBody latches its last command, so hold zero.
		body.set_command(0.0, 0.0)
		return
	bb.time += dt
	bb.position = body.global_position
	bb.yaw = body.yaw()
	for s: SensorBase in _sensors:
		if s.due(dt):
			s.sense(bb)
	# Typed local: a ternary `detect(...) if cond else []` yields an untyped Array and
	# fails at runtime when assigned to Array[Dictionary].
	var blobs: Array[Dictionary] = []
	if bb.lidar_enabled:
		blobs = LidarBlobs.detect(bb.lidar_ranges, bb.lidar_angles, bb.lidar_max_range)
		blobs = LidarBlobs.off_walls(blobs, bb, _walls)
	bb.blobs = blobs
	if caught:
		body.set_command(0.0, 0.0)
		return
	fsm.update(bb, dt)
	var cmd: Vector2 = motion.update(bb)
	body.set_command(cmd.x, cmd.y)
	if bb.stuck and not _was_stuck:
		stuck_events += 1
	_was_stuck = bb.stuck
	if _overlay:
		_overlay.draw(bb, _vision)
