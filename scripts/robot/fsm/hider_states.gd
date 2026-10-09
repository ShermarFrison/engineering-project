## Hider decision logic: PickSpot -> GoToSpot -> Hide; Hide -> Flee / Relocate.
class_name HiderStates
extends RefCounted


class Context extends RefCounted:
	var grid: VisibilityGrid
	var seeker_spawn: Vector3 = Vector3.ZERO
	var strategy: HiderStrategy
	var flee_clear_timeout: float = 4.0
	var threat_blob_dist: float = 4.0
	var min_hide_dist: float = 8.0
	var last_relocate_time: float = 0.0
	var flee_refresh: float = 1.0


static func threat_blob(bb: Blackboard, ctx: Context) -> bool:
	for blob: Dictionary in bb.blobs:
		if blob["distance"] < ctx.threat_blob_dist:
			return true
	return false


class PickSpot extends FSMState:
	var ctx: Context

	func _init(c: Context) -> void:
		name = "PickSpot"
		ctx = c

	func enter(bb: Blackboard) -> void:
		bb.goal_pos = ctx.strategy.pick_spot(bb, ctx)
		bb.goal_active = true

	func update(_bb: Blackboard, _dt: float) -> String:
		reason = "spot chosen"
		return "GoToSpot"


class GoToSpot extends FSMState:
	func _init() -> void:
		name = "GoToSpot"

	func update(bb: Blackboard, _dt: float) -> String:
		if not bb.seen_seeker().is_empty():
			reason = "seeker seen"
			return "Flee"
		if bb.goal_reached:
			reason = "arrived"
			return "Hide"
		if bb.stuck:
			reason = "stuck"
			return "PickSpot"
		return ""


class Hide extends FSMState:
	var ctx: Context

	func _init(c: Context) -> void:
		name = "Hide"
		ctx = c

	func enter(bb: Blackboard) -> void:
		bb.goal_active = false

	func update(bb: Blackboard, _dt: float) -> String:
		if not bb.seen_seeker().is_empty():
			reason = "seeker seen"
			return "Flee"
		if HiderStates.threat_blob(bb, ctx):
			reason = "lidar blob near"
			return "Relocate"
		if ctx.strategy.wants_relocate(bb, ctx):
			reason = "strategy relocate"
			return "Relocate"
		return ""


class Flee extends FSMState:
	var ctx: Context
	var _last_refresh: float = -INF

	func _init(c: Context) -> void:
		name = "Flee"
		ctx = c

	func _refresh(bb: Blackboard) -> void:
		var threat: Vector3 = bb.seeker_last_seen_pos
		bb.goal_pos = ctx.grid.pick_flee_spot(bb.position, threat)
		bb.goal_active = true
		_last_refresh = bb.time

	func enter(bb: Blackboard) -> void:
		_refresh(bb)

	func update(bb: Blackboard, _dt: float) -> String:
		if bb.time_since_seeker_seen() > ctx.flee_clear_timeout:
			reason = "seeker unseen"
			return "PickSpot"
		if (not bb.seen_seeker().is_empty() and bb.time - _last_refresh > ctx.flee_refresh) or bb.goal_reached or bb.stuck:
			_refresh(bb)
		return ""


class Relocate extends FSMState:
	var ctx: Context

	func _init(c: Context) -> void:
		name = "Relocate"
		ctx = c

	func enter(bb: Blackboard) -> void:
		ctx.last_relocate_time = bb.time
		bb.goal_pos = ctx.strategy.pick_spot(bb, ctx)
		bb.goal_active = true

	func update(_bb: Blackboard, _dt: float) -> String:
		reason = "new spot"
		return "GoToSpot"


static func build(ctx: Context) -> FSM:
	var fsm := FSM.new()
	fsm.add(PickSpot.new(ctx))
	fsm.add(GoToSpot.new())
	fsm.add(Hide.new(ctx))
	fsm.add(Flee.new(ctx))
	fsm.add(Relocate.new(ctx))
	return fsm
