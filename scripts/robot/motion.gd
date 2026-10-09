## Goal -> (v, w). Global path from path_provider, local heading from Steering,
## ultrasonic slow-down, bumper recovery, stuck detection.
class_name Motion
extends RefCounted

var path_provider: Callable  # (from: Vector3, to: Vector3) -> PackedVector3Array

var max_v: float = 2.0
var max_w: float = 3.0
var k_ang: float = 2.5
var waypoint_radius: float = 0.6
var arrive_radius: float = 0.5
var replan_period: float = 2.0
var stuck_window: float = 4.0
var stuck_progress: float = 0.3
var recovery_time: float = 0.8
var ultrasonic_slow_dist: float = 0.6
var avoid_dist: float = 1.5  # Steering.pick_heading proximity threshold (tuning knob)

var path: PackedVector3Array = PackedVector3Array()
var _idx: int = 0
var _goal: Vector3 = Vector3.ZERO
var _has_goal: bool = false
var _last_replan: float = -INF
var _progress_dist: float = INF
var _progress_time: float = 0.0
var _recover_until: float = -INF
var _recover_w: float = 0.0


func reset() -> void:
	path = PackedVector3Array()
	_idx = 0
	_has_goal = false
	_progress_dist = INF
	_recover_until = -INF


func _replan(bb: Blackboard) -> void:
	path = path_provider.call(bb.position, _goal) if path_provider.is_valid() else PackedVector3Array([bb.position, _goal])
	_idx = 0
	_last_replan = bb.time


func update(bb: Blackboard) -> Vector2:
	bb.goal_reached = false
	if not bb.goal_active:
		bb.stuck = false
		reset()
		return Vector2.ZERO

	# New goal?
	if not _has_goal or _goal.distance_to(bb.goal_pos) > 0.5:
		_goal = bb.goal_pos
		_has_goal = true
		_replan(bb)
		_progress_dist = bb.position.distance_to(_goal)
		_progress_time = bb.time
		bb.stuck = false

	var dist_goal: float = bb.position.distance_to(_goal)
	if dist_goal < arrive_radius:
		bb.goal_reached = true
		bb.stuck = false
		return Vector2.ZERO

	# Stuck detection on progress toward the goal.
	if dist_goal < _progress_dist - stuck_progress:
		_progress_dist = dist_goal
		_progress_time = bb.time
		bb.stuck = false
	elif bb.time - _progress_time > stuck_window:
		bb.stuck = true
		_replan(bb)
		_progress_dist = dist_goal
		_progress_time = bb.time
	else:
		bb.stuck = false

	# Bumper recovery: back off and turn away from the contact.
	if bb.bumper_enabled and bb.bumper_hit and bb.time >= _recover_until:
		_recover_until = bb.time + recovery_time
		var n_angle: float = bb.world_to_local_angle(bb.position + bb.bumper_normal)
		_recover_w = -1.5 if n_angle > 0.0 else 1.5
	if bb.time < _recover_until:
		return Vector2(-0.6, _recover_w)

	# Periodic replan keeps the path fresh if the goal moves slowly.
	if bb.time - _last_replan > replan_period:
		_replan(bb)

	# Waypoint following.
	while _idx < path.size() - 1 and bb.position.distance_to(path[_idx]) < waypoint_radius:
		_idx += 1
	var target: Vector3 = path[_idx] if path.size() > 0 else _goal
	var target_angle: float = bb.world_to_local_angle(target)
	var heading: float = target_angle
	var front: float = INF
	if bb.lidar_enabled and bb.lidar_ranges.size() > 0:
		heading = Steering.pick_heading(target_angle, bb.lidar_ranges, bb.lidar_angles, bb.lidar_max_range, avoid_dist)
		front = bb.min_range_in_arc(0.0, deg_to_rad(20.0))
		if front >= bb.lidar_max_range - 1e-3:
			front = INF

	var w: float = clampf(k_ang * heading, -max_w, max_w)
	var align: float = clampf(1.0 - absf(heading) / (PI * 0.5), 0.0, 1.0)
	var v: float = max_v * Steering.speed_scale(front) * align
	if bb.ultrasonic_enabled and bb.ultrasonic.size() > 0 and bb.ultrasonic[0] < ultrasonic_slow_dist:
		v = minf(v, max_v * 0.5 * bb.ultrasonic[0] / ultrasonic_slow_dist)
	return Vector2(v, w)
