## Base for simulated sensors. Subclasses override sense(bb).
class_name SensorBase
extends Node3D

@export var enabled: bool = true
@export var noise_std: float = 0.0
@export var update_hz: float = 60.0  # samples per second; <= 0 means every tick

var body: DriveBody

var _accum: float = 0.0


func sense(_bb: Blackboard) -> void:
	pass


## Rate limiter, called by RobotBrain once per physics tick. Returns true when a
## sample is owed at update_hz. At the default 60 Hz with 60 Hz ticks this is
## true every tick, so tests that call sense() directly are unaffected.
func due(dt: float) -> bool:
	if update_hz <= 0.0:
		return true
	var period: float = 1.0 / update_hz
	_accum += dt
	if _accum + 1e-6 < period:
		return false
	_accum -= period
	if _accum > period:
		_accum = 0.0  # fell far behind (huge dt): do not burst-sample
	return true


func _noise(bb: Blackboard) -> float:
	if noise_std <= 0.0:
		return 0.0
	return bb.rng.randfn(0.0, noise_std)


func _space() -> PhysicsDirectSpaceState3D:
	return get_world_3d().direct_space_state
