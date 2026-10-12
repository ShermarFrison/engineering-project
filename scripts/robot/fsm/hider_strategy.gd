## Swappable hiding-spot policy used by the hider FSM.
class_name HiderStrategy
extends RefCounted

const TOP_K := 6  # 3 -> 6 in the final-review fix wave: hiders converged on the same nook

var kind: String = "base"


static func create(kind: String) -> HiderStrategy:
	match kind:
		"dynamic":
			return DynamicHider.new()
		"static":
			return StaticHider.new()
		_:
			push_error("unknown hider strategy: " + kind)
			return StaticHider.new()


## Random pick among the top-K hiding spots, skipping those within avoid_radius of avoid
## and those within ctx.hider_spacing of another hider this one has seen (bb.last_known,
## so only hiders it knows about). Falls back to the full top-K list if nothing is left.
static func choose_spot(bb: Blackboard, ctx: HiderStates.Context, avoid: Vector3, avoid_radius: float) -> Vector3:
	var threat: Vector3 = bb.seeker_last_seen_pos if bb.seeker_last_seen_time > -INF else ctx.seeker_spawn
	var all: Array[Vector3] = ctx.grid.pick_hiding_spots(threat, ctx.min_hide_dist, TOP_K)
	var others: Array[Vector3] = []
	if ctx.hider_spacing > 0.0:
		for id: int in bb.last_known:
			var rec: Dictionary = bb.last_known[id]
			if rec["role"] == "hider":
				others.append(rec["position"])
	var cands: Array[Vector3] = []
	for c: Vector3 in all:
		if c.distance_to(avoid) < avoid_radius:
			continue
		var crowded: bool = false
		for o: Vector3 in others:
			if Vector2(c.x, c.z).distance_to(Vector2(o.x, o.z)) < ctx.hider_spacing:
				crowded = true
				break
		if not crowded:
			cands.append(c)
	if cands.is_empty():
		cands = all
	return cands[bb.rng.randi_range(0, cands.size() - 1)]


func pick_spot(bb: Blackboard, ctx: HiderStates.Context) -> Vector3:
	return choose_spot(bb, ctx, bb.position, 0.0)


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
		return HiderStrategy.choose_spot(bb, ctx, bb.position, 1.5)

	func wants_relocate(bb: Blackboard, ctx: HiderStates.Context) -> bool:
		return bb.time - ctx.last_relocate_time >= relocate_period
