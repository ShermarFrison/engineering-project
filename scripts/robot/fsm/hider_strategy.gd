## Swappable hiding-spot policy used by the hider FSM.
class_name HiderStrategy
extends RefCounted

var kind: String = "base"


static func create(kind: String) -> HiderStrategy:
	match kind:
		"dynamic":
			return DynamicHider.new()
		_:
			return StaticHider.new()


func _threat(bb: Blackboard, ctx: HiderStates.Context) -> Vector3:
	return bb.seeker_last_seen_pos if bb.seeker_last_seen_time > -INF else ctx.seeker_spawn


func pick_spot(bb: Blackboard, ctx: HiderStates.Context) -> Vector3:
	return ctx.grid.pick_hiding_spot(_threat(bb, ctx), ctx.min_hide_dist)


func wants_relocate(_bb: Blackboard, _ctx: HiderStates.Context) -> bool:
	return false


## Picks the least visible spot far from the seeker's spawn and stays there.
class StaticHider extends HiderStrategy:
	func _init() -> void:
		kind = "static"


## Periodically moves to a new spot far from the seeker's last known position.
class DynamicHider extends HiderStrategy:
	var relocate_period: float = 20.0

	func _init() -> void:
		kind = "dynamic"

	func pick_spot(bb: Blackboard, ctx: HiderStates.Context) -> Vector3:
		var threat: Vector3 = _threat(bb, ctx)
		var spot: Vector3 = ctx.grid.pick_hiding_spot(threat, ctx.min_hide_dist)
		if spot.distance_to(bb.position) < 1.5:
			spot = ctx.grid.pick_flee_spot(bb.position, threat)
		return spot

	func wants_relocate(bb: Blackboard, ctx: HiderStates.Context) -> bool:
		return bb.time - ctx.last_relocate_time >= relocate_period
