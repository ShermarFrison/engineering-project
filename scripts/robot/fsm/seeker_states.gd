## Seeker decision logic: Wait -> Patrol <-> Investigate / Chase -> Catch -> Patrol.
## All states read only the Blackboard and the shared Context. Every transition
## sets `reason`, which FSM logs with the round time.
class_name SeekerStates
extends RefCounted


class Context extends RefCounted:
	var grid: VisibilityGrid
	var los: Callable
	var tag_range: float = 1.5
	var lose_timeout: float = 3.0
	var investigate_timeout: float = 8.0
	var catch_delay: float = 0.5
	var fov: float = deg_to_rad(90.0)
	var vision_range: float = 10.0
	var local_mark_radius: float = 2.5
	## Recently investigated spots, each {"pos": Vector3, "time": float}; ignored for investigate_cooldown.
	var investigated: Array[Dictionary] = []
	var investigate_cooldown: float = 10.0
	## Without vision, a blob this close counts as a detection: Patrol/Investigate -> Chase.
	var chase_blob_dist: float = 3.0
	# chase memory
	var chase_last_pos: Vector3 = Vector3.ZERO
	var chase_last_seen: float = -INF
	# blob-only catch that caught nobody (prop or wall end); ignored for investigate_cooldown
	var duds: Array[Dictionary] = []


static func _remember(list: Array[Dictionary], pos: Vector3, now: float, ttl: float) -> void:
	var i: int = list.size() - 1
	while i >= 0:
		if now - float(list[i]["time"]) >= ttl:
			list.remove_at(i)
		i -= 1
	list.append({"pos": pos, "time": now})


static func _near_any(list: Array[Dictionary], p: Vector3, radius: float, now: float, ttl: float) -> bool:
	for e: Dictionary in list:
		if now - float(e["time"]) < ttl and p.distance_to(e["pos"]) < radius:
			return true
	return false


static func blob_world_pos(bb: Blackboard, blob: Dictionary) -> Vector3:
	return bb.position + bb.local_to_world_dir(blob["angle"]) * blob["distance"]


## Nearest blob worth reacting to: not where a caught hider sits, not a recent dud and,
## when respect_cooldown, not the spot investigated within investigate_cooldown.
## Chase decisions pass respect_cooldown = false so the blob being investigated can
## still be chased once the seeker is close to it.
static func interesting_blob(bb: Blackboard, ctx: Context, respect_cooldown: bool = true) -> Dictionary:
	var best: Dictionary = {}
	var best_d: float = INF
	for blob: Dictionary in bb.blobs:
		var p: Vector3 = blob_world_pos(bb, blob)
		var skip: bool = false
		for id: int in bb.caught_ids:
			if bb.last_known.has(id) and bb.last_known[id]["position"].distance_to(p) < 1.5:
				skip = true
				break
		if skip:
			continue
		if SeekerStates._near_any(ctx.duds, p, 1.0, bb.time, ctx.investigate_cooldown):
			continue
		if respect_cooldown and SeekerStates._near_any(ctx.investigated, p, 2.0, bb.time, ctx.investigate_cooldown):
			continue
		if blob["distance"] < best_d:
			best_d = blob["distance"]
			best = blob
	return best


## Blind fallback (spec section 4): without vision, a blob within chase_blob_dist is a detection.
static func near_blob_without_vision(bb: Blackboard, ctx: Context) -> bool:
	if bb.vision_enabled:
		return false
	var blob: Dictionary = interesting_blob(bb, ctx, false)
	return not blob.is_empty() and blob["distance"] <= ctx.chase_blob_dist


static func nearest_seen_hider(bb: Blackboard) -> Dictionary:
	var best: Dictionary = {}
	for s: Dictionary in bb.seen_hiders():
		if best.is_empty() or s["distance"] < best["distance"]:
			best = s
	return best


static func mark_viewed(bb: Blackboard, ctx: Context) -> void:
	if ctx.grid == null:
		return
	if bb.vision_enabled:
		ctx.grid.mark_viewed(bb.position, bb.yaw, ctx.fov, ctx.vision_range, bb.time, ctx.los)
	ctx.grid.mark_viewed(bb.position, bb.yaw, TAU, ctx.local_mark_radius, bb.time, ctx.los)


class Wait extends FSMState:
	func _init() -> void:
		name = "Wait"

	func enter(bb: Blackboard) -> void:
		bb.goal_active = false

	func update(bb: Blackboard, _dt: float) -> String:
		if bb.seeking_allowed:
			reason = "head start over"
			return "Patrol"
		return ""


class Patrol extends FSMState:
	var ctx: Context

	func _init(c: Context) -> void:
		name = "Patrol"
		ctx = c

	func _pick(bb: Blackboard) -> void:
		bb.goal_pos = ctx.grid.pick_patrol_target(bb.position, bb.time)
		bb.goal_active = true

	func enter(bb: Blackboard) -> void:
		_pick(bb)

	func update(bb: Blackboard, _dt: float) -> String:
		SeekerStates.mark_viewed(bb, ctx)
		if not SeekerStates.nearest_seen_hider(bb).is_empty():
			reason = "hider seen"
			return "Chase"
		if SeekerStates.near_blob_without_vision(bb, ctx):
			reason = "near blob, no vision"
			return "Chase"
		if not SeekerStates.interesting_blob(bb, ctx).is_empty():
			reason = "lidar blob"
			return "Investigate"
		if bb.goal_reached or bb.stuck:
			_pick(bb)
		return ""


class Investigate extends FSMState:
	var ctx: Context
	var _started: float = 0.0
	var _arrived_at: float = -INF

	func _init(c: Context) -> void:
		name = "Investigate"
		ctx = c

	func enter(bb: Blackboard) -> void:
		_started = bb.time
		_arrived_at = -INF
		var blob: Dictionary = SeekerStates.interesting_blob(bb, ctx)
		var target: Vector3
		if not blob.is_empty():
			target = SeekerStates.blob_world_pos(bb, blob)
		elif ctx.chase_last_seen > -INF and bb.time - ctx.chase_last_seen < 30.0:
			target = ctx.chase_last_pos
		else:
			target = bb.position
		SeekerStates._remember(ctx.investigated, target, bb.time, ctx.investigate_cooldown)
		bb.goal_pos = target
		bb.goal_active = true

	func update(bb: Blackboard, _dt: float) -> String:
		SeekerStates.mark_viewed(bb, ctx)
		if not SeekerStates.nearest_seen_hider(bb).is_empty():
			reason = "hider seen"
			return "Chase"
		if SeekerStates.near_blob_without_vision(bb, ctx):
			reason = "near blob, no vision"
			return "Chase"
		if bb.time - _started > ctx.investigate_timeout:
			reason = "timeout"
			return "Patrol"
		if bb.stuck:
			reason = "stuck"
			return "Patrol"
		if bb.goal_reached:
			if _arrived_at < 0.0:
				_arrived_at = bb.time
			elif bb.time - _arrived_at > 1.5:
				reason = "nothing found"
				return "Patrol"
		return ""


class Chase extends FSMState:
	var ctx: Context
	var _entered: float = 0.0

	func _init(c: Context) -> void:
		name = "Chase"
		ctx = c

	func enter(bb: Blackboard) -> void:
		_entered = bb.time
		ctx.chase_last_seen = -INF
		var seen: Dictionary = SeekerStates.nearest_seen_hider(bb)
		if not seen.is_empty():
			bb.goal_pos = seen["position"]
		else:
			var blob: Dictionary = SeekerStates.interesting_blob(bb, ctx, false)
			if not bb.vision_enabled and not blob.is_empty():
				bb.goal_pos = SeekerStates.blob_world_pos(bb, blob)
			else:
				bb.goal_pos = bb.position
		bb.goal_active = true

	## Shared pursuit step for a seen hider (request = its id) or, without vision, a
	## blob (request = -1): remember it, steer at it, request a catch inside tag range.
	func _pursue(bb: Blackboard, target_pos: Vector3, request: int) -> String:
		ctx.chase_last_pos = target_pos
		ctx.chase_last_seen = bb.time
		bb.goal_pos = target_pos
		bb.goal_active = true
		var d: float = Vector2(bb.position.x, bb.position.z).distance_to(Vector2(target_pos.x, target_pos.z))
		if d < ctx.tag_range:
			bb.catch_request = request
			reason = "in tag range"
			return "Catch"
		return ""

	func update(bb: Blackboard, _dt: float) -> String:
		SeekerStates.mark_viewed(bb, ctx)
		var target: Dictionary = SeekerStates.nearest_seen_hider(bb)
		if not target.is_empty():
			return _pursue(bb, target["position"], target["id"])
		if not bb.vision_enabled:
			var blob: Dictionary = SeekerStates.interesting_blob(bb, ctx, false)
			if not blob.is_empty():
				return _pursue(bb, SeekerStates.blob_world_pos(bb, blob), -1)
		# Target lost (or never acquired in this Chase): back to Patrol (spec).
		if ctx.chase_last_seen == -INF:
			if bb.time - _entered > ctx.lose_timeout:
				reason = "nothing to chase"
				return "Patrol"
			return ""
		if bb.time - ctx.chase_last_seen > ctx.lose_timeout:
			reason = "target lost"
			return "Patrol"
		return ""


class Catch extends FSMState:
	var ctx: Context
	var _entered: float = 0.0
	var _caught_before: int = 0

	func _init(c: Context) -> void:
		name = "Catch"
		ctx = c

	func enter(bb: Blackboard) -> void:
		_entered = bb.time
		_caught_before = bb.caught_ids.size()
		bb.goal_active = false

	func update(bb: Blackboard, _dt: float) -> String:
		if bb.time - _entered >= ctx.catch_delay:
			reason = "catch done"
			return "Patrol"
		return ""

	func exit(bb: Blackboard) -> void:
		if bb.catch_request == -1 and bb.caught_ids.size() == _caught_before:
			# Blind catch found no hider: the blob was a prop or a wall end. Ignore it for a while.
			SeekerStates._remember(ctx.duds, ctx.chase_last_pos, bb.time, ctx.investigate_cooldown)
		bb.catch_request = 0


static func build(ctx: Context) -> FSM:
	var fsm := FSM.new()
	fsm.add(Wait.new())
	fsm.add(Patrol.new(ctx))
	fsm.add(Investigate.new(ctx))
	fsm.add(Chase.new(ctx))
	fsm.add(Catch.new(ctx))
	return fsm
