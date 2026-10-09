## Differential-drive chassis. Applies DriveModel wheel speeds as body velocity
## inside _integrate_forces so collisions with props and robots stay physical.
## Requires a frictionless physics material (set in robot.tscn); see NOTES.
class_name DriveBody
extends RigidBody3D

@export var role: String = "hider"  # "seeker" | "hider"
@export var wheel_radius: float = 0.1
@export var track_width: float = 0.4
@export var max_wheel_speed: float = 25.0
@export var max_wheel_accel: float = 60.0

var model: DriveModel = DriveModel.new()
var distance_travelled: float = 0.0
var collision_count: int = 0
var contacts: Array[Dictionary] = []

var _cmd_v: float = 0.0
var _cmd_w: float = 0.0
var _had_contact: bool = false
var _last_pos: Vector3

# Visual wheel spin: rest basis from the scene plus an accumulated axle angle.
var _wheel_l: Node3D
var _wheel_r: Node3D
var _wheel_l_rest: Basis = Basis.IDENTITY
var _wheel_r_rest: Basis = Basis.IDENTITY
var _spin_l: float = 0.0
var _spin_r: float = 0.0


func _ready() -> void:
	model.wheel_radius = wheel_radius
	model.track_width = track_width
	model.max_wheel_speed = max_wheel_speed
	model.max_wheel_accel = max_wheel_accel
	add_to_group("robots")
	contact_monitor = true
	max_contacts_reported = 8
	axis_lock_angular_x = true
	axis_lock_angular_z = true
	_last_pos = global_position
	_wheel_l = get_node_or_null("WheelL") as Node3D
	_wheel_r = get_node_or_null("WheelR") as Node3D
	if _wheel_l:
		_wheel_l_rest = _wheel_l.transform.basis
	if _wheel_r:
		_wheel_r_rest = _wheel_r.transform.basis


## Visual only: spin the wheel meshes about their axle at the current wheel speeds.
## The cylinder's local Y is the axle (robot X); positive wheel speed drives toward -Z,
## which is a negative rotation about +X.
func _physics_process(dt: float) -> void:
	_spin_l = wrapf(_spin_l - model.wl * dt, -PI, PI)
	_spin_r = wrapf(_spin_r - model.wr * dt, -PI, PI)
	if _wheel_l:
		_wheel_l.transform.basis = _wheel_l_rest * Basis(Vector3.UP, _spin_l)
	if _wheel_r:
		_wheel_r.transform.basis = _wheel_r_rest * Basis(Vector3.UP, _spin_r)


func set_command(v: float, w: float) -> void:
	_cmd_v = v
	_cmd_w = w


func yaw() -> float:
	return global_rotation.y


func forward() -> Vector3:
	return -global_transform.basis.z


func _integrate_forces(state: PhysicsDirectBodyState3D) -> void:
	var dt: float = state.step
	model.command(_cmd_v, _cmd_w, dt)
	var vw: Vector2 = model.body_velocity()
	var lv: Vector3 = forward() * vw.x
	lv.y = state.linear_velocity.y  # keep gravity
	state.linear_velocity = lv
	state.angular_velocity = Vector3(0.0, vw.y, 0.0)

	# Contacts (world space), filtered to roughly horizontal normals = not the floor.
	contacts.clear()
	for i: int in state.get_contact_count():
		var n: Vector3 = state.get_contact_local_normal(i)
		if absf(n.y) > 0.5:
			continue
		contacts.append({
			"normal": n,
			"position": state.get_contact_local_position(i),
		})
	var has_contact: bool = not contacts.is_empty()
	if has_contact and not _had_contact:
		collision_count += 1
	_had_contact = has_contact

	# Odometry (horizontal distance only).
	var p: Vector3 = state.transform.origin
	var d: Vector3 = p - _last_pos
	d.y = 0.0
	distance_travelled += d.length()
	_last_pos = p
