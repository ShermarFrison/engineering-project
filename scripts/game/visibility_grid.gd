## Coarse XZ grid over the arena. Knows, per walkable cell, how open it is
## (fraction of rays reaching open_range against walls), whether the seeker's
## spawn can see it, and when the seeker last viewed it. Pure: LOS is injected.
class_name VisibilityGrid
extends RefCounted

var cell_size: float = 2.0
var centers: Array[Vector3] = []
var openness: PackedFloat32Array = PackedFloat32Array()
var visible_from_spawn: Array[bool] = []
var last_viewed: PackedFloat32Array = PackedFloat32Array()
var wall_rects: Array[Rect2] = []  # static walls given to build(); used to drop wall-end lidar blobs

const NEVER: float = -1000.0


func build(bounds: Rect2, walls: Array[Rect2], seeker_spawn: Vector3, los: Callable,
		open_range: float = 4.0, ray_count: int = 16) -> void:
	centers.clear()
	openness = PackedFloat32Array()
	visible_from_spawn.clear()
	last_viewed = PackedFloat32Array()
	wall_rects = walls.duplicate()
	# Grown by 1 mm so cells centred exactly on the far edge count: Rect2.has_point excludes
	# the end edge, which would drop the whole +X column and +Z row.
	var inner: Rect2 = bounds.grow(-cell_size * 0.5).grow(0.001)
	var nx: int = int(floor(bounds.size.x / cell_size))
	var nz: int = int(floor(bounds.size.y / cell_size))
	for ix: int in nx:
		for iz: int in nz:
			var p := Vector2(bounds.position.x + (ix + 0.5) * cell_size, bounds.position.y + (iz + 0.5) * cell_size)
			if not inner.has_point(p):
				continue
			var blocked: bool = false
			for w: Rect2 in walls:
				if w.grow(0.5).has_point(p):
					blocked = true
					break
			if blocked:
				continue
			var c := Vector3(p.x, 0.0, p.y)
			centers.append(c)
			var open: int = 0
			for k: int in ray_count:
				var a: float = TAU * k / ray_count
				var to: Vector3 = c + Vector3(-sin(a), 0.0, -cos(a)) * open_range
				if los.call(c, to):
					open += 1
			openness.append(float(open) / ray_count)
			visible_from_spawn.append(bool(los.call(seeker_spawn, c)))
			last_viewed.append(NEVER)


func nearest_cell(p: Vector3) -> int:
	var best: int = -1
	var best_d: float = INF
	for i: int in centers.size():
		var d: float = centers[i].distance_squared_to(p)
		if d < best_d:
			best_d = d
			best = i
	return best


func mark_viewed(pos: Vector3, yaw: float, fov: float, range: float, now: float, los: Callable) -> int:
	var marked: int = 0
	var half: float = fov * 0.5
	for i: int in centers.size():
		var d: Vector3 = centers[i] - pos
		if d.length() > range:
			continue
		var ang: float = wrapf(atan2(-d.x, -d.z) - yaw, -PI, PI)
		if absf(ang) > half:
			continue
		if los.call(pos, centers[i]):
			last_viewed[i] = now
			marked += 1
	return marked


func pick_patrol_target(from: Vector3, now: float) -> Vector3:
	var best: int = 0
	var best_s: float = -INF
	for i: int in centers.size():
		var stale: float = minf(now - last_viewed[i], 60.0)
		var s: float = stale - 0.5 * from.distance_to(centers[i])
		if s > best_s:
			best_s = s
			best = i
	return centers[best]


func pick_hiding_spot(threat: Vector3, min_threat_dist: float) -> Vector3:
	var best: int = -1
	var best_s: float = -INF
	var min_d: float = min_threat_dist
	while best == -1 and min_d >= 0.0:
		for i: int in centers.size():
			var dt: float = centers[i].distance_to(threat)
			if dt < min_d:
				continue
			var s: float = -3.0 * openness[i] - (2.0 if visible_from_spawn[i] else 0.0) + 0.05 * dt
			if s > best_s:
				best_s = s
				best = i
		min_d -= 2.0  # relax if nothing qualified
	return centers[maxi(best, 0)]


func pick_flee_spot(from: Vector3, threat: Vector3) -> Vector3:
	var best: int = -1
	var best_s: float = -INF
	var d_from: float = from.distance_to(threat)
	for i: int in centers.size():
		var dt: float = centers[i].distance_to(threat)
		var df: float = centers[i].distance_to(from)
		if df > 10.0 or dt <= d_from:
			continue
		var s: float = dt - 0.5 * df - 2.0 * openness[i]
		if s > best_s:
			best_s = s
			best = i
	if best == -1:  # fall back: farthest cell from threat
		for i: int in centers.size():
			var dt: float = centers[i].distance_to(threat)
			if dt > best_s:
				best_s = dt
				best = i
	return centers[best]
