## Per-robot blackboard: everything the decision and motion layers may read.
## Sensors write here; nothing else reads the scene tree.
class_name Blackboard
extends RefCounted

# --- robot state (written by RobotBrain each tick)
var time: float = 0.0
var position: Vector3 = Vector3.ZERO
var yaw: float = 0.0
var rng: RandomNumberGenerator = RandomNumberGenerator.new()
var role: String = "hider"
var caught_ids: Array[int] = []

# --- lidar
var lidar_enabled: bool = false
var lidar_ranges: PackedFloat32Array = PackedFloat32Array()
var lidar_angles: PackedFloat32Array = PackedFloat32Array()
var lidar_max_range: float = 8.0

# --- vision
var vision_enabled: bool = false
var seen_robots: Array[Dictionary] = []
var last_known: Dictionary = {}  # id -> {position, time, role}
var seeker_last_seen_pos: Vector3 = Vector3.ZERO
var seeker_last_seen_time: float = -INF

# --- ultrasonic / bumper
var ultrasonic_enabled: bool = false
var ultrasonic: PackedFloat32Array = PackedFloat32Array([INF, INF, INF])
var bumper_enabled: bool = false
var bumper_hit: bool = false
var bumper_normal: Vector3 = Vector3.ZERO

# --- derived
var blobs: Array[Dictionary] = []

# --- motion feedback
var stuck: bool = false
var goal_reached: bool = false

# --- FSM outputs
var goal_pos: Vector3 = Vector3.ZERO
var goal_active: bool = false
var catch_request: int = 0
var state_name: String = ""


func local_to_world_dir(angle: float) -> Vector3:
	var a: float = yaw + angle
	return Vector3(-sin(a), 0.0, -cos(a))


func world_to_local_angle(p: Vector3) -> float:
	var d: Vector3 = p - position
	var world_angle: float = atan2(-d.x, -d.z)
	return wrapf(world_angle - yaw, -PI, PI)


func nearest_obstacle() -> Dictionary:
	var best_d: float = INF
	var best_a: float = 0.0
	for i: int in lidar_ranges.size():
		if lidar_ranges[i] < lidar_max_range - 1e-3 and lidar_ranges[i] < best_d:
			best_d = lidar_ranges[i]
			best_a = lidar_angles[i]
	return {"distance": best_d, "angle": best_a}


func min_range_in_arc(center: float, half_width: float) -> float:
	var best: float = INF
	for i: int in lidar_ranges.size():
		if absf(wrapf(lidar_angles[i] - center, -PI, PI)) <= half_width + 1e-6:
			best = minf(best, lidar_ranges[i])
	return best


func time_since_seeker_seen() -> float:
	return time - seeker_last_seen_time


func seen_hiders() -> Array[Dictionary]:
	var out: Array[Dictionary] = []
	for s: Dictionary in seen_robots:
		if s["role"] == "hider" and not caught_ids.has(s["id"]):
			out.append(s)
	return out


func seen_seeker() -> Dictionary:
	for s: Dictionary in seen_robots:
		if s["role"] == "seeker":
			return s
	return {}
