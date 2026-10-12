## Clusters lidar hits into free-standing, robot-sized blobs.
## This is the seeker's fallback detector when vision is disabled.
class_name LidarBlobs
extends RefCounted


## min_width 0.0: with a 10 degree lidar a robot's 0.3 m mast is often a single ray.
## max_width 0.5: chords never exceed true width, so every mast (<= 0.42 m) passes,
## while 1.0 m crate faces seen by enough rays exceed it.
static func detect(ranges: PackedFloat32Array, angles: PackedFloat32Array, max_range: float,
		min_width: float = 0.0, max_width: float = 0.5, break_gap: float = 0.5) -> Array[Dictionary]:
	var out: Array[Dictionary] = []
	var n: int = ranges.size()
	if n < 3:
		return out
	var hit := func(i: int) -> bool: return ranges[posmod(i, n)] < max_range - 1e-3
	# Start at a cluster boundary (unhit ray, unhit predecessor, or a range jump) so wraparound works.
	var start: int = -1
	for s_i: int in n:
		var prev: int = posmod(s_i - 1, n)
		if not hit.call(s_i) or not hit.call(prev) or absf(ranges[s_i] - ranges[prev]) >= break_gap:
			start = s_i
			break
	if start == -1:
		return out  # uniform ring: no boundary, no blobs
	var i: int = start
	var visited: int = 0
	while visited < n:
		var k: int = posmod(i, n)
		if not hit.call(k):
			i += 1
			visited += 1
			continue
		# cluster from k while consecutive range differences are small
		var idxs: Array[int] = [k]
		var j: int = i + 1
		while visited + idxs.size() < n and hit.call(j) and absf(ranges[posmod(j, n)] - ranges[posmod(j - 1, n)]) < break_gap:
			idxs.append(posmod(j, n))
			j += 1
		visited += idxs.size()
		i = j
		# free-standing check: neighbours either unhit or far behind
		var before: int = posmod(idxs[0] - 1, n)
		var after: int = posmod(idxs[-1] + 1, n)
		var first_r: float = ranges[idxs[0]]
		var last_r: float = ranges[idxs[-1]]
		var free_before: bool = (not hit.call(before)) or ranges[before] - first_r >= break_gap
		var free_after: bool = (not hit.call(after)) or ranges[after] - last_r >= break_gap
		if not (free_before and free_after):
			continue
		var p0 := Vector2(first_r * cos(angles[idxs[0]]), first_r * sin(angles[idxs[0]]))
		var p1 := Vector2(last_r * cos(angles[idxs[-1]]), last_r * sin(angles[idxs[-1]]))
		var width: float = p0.distance_to(p1)
		if width < min_width or width > max_width:
			continue
		var sum_d: float = 0.0
		var sx: float = 0.0
		var sy: float = 0.0
		for idx: int in idxs:
			sum_d += ranges[idx]
			sx += cos(angles[idx])
			sy += sin(angles[idx])
		out.append({
			"angle": atan2(sy, sx),
			"distance": sum_d / idxs.size(),
			"width": width,
		})
	return out


## Drops blobs lying on a known static wall (grown by margin). A wall end seen
## end-on looks like a free-standing 0.3 m object; robots know the static layout.
static func off_walls(blobs: Array[Dictionary], bb: Blackboard, walls: Array[Rect2],
		margin: float = 0.1) -> Array[Dictionary]:
	var out: Array[Dictionary] = []
	for b: Dictionary in blobs:
		var p: Vector3 = bb.position + bb.local_to_world_dir(b["angle"]) * b["distance"]
		var on_wall: bool = false
		for w: Rect2 in walls:
			if w.grow(margin).has_point(Vector2(p.x, p.z)):
				on_wall = true
				break
		if not on_wall:
			out.append(b)
	return out
