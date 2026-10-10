## Seeded random placement of rectangular props on the XZ plane.
## Rejects candidates overlapping walls, other props, or keep-clear points,
## with a bounded attempt budget so crowded arenas terminate.
class_name PropLayout
extends RefCounted


static func overlaps(a: Rect2, b: Rect2, gap: float) -> bool:
	return a.grow(gap * 0.5).intersects(b.grow(gap * 0.5))


static func _footprint(pos: Vector2, size: Vector2, rot: float) -> Rect2:
	var s: Vector2 = size if rot == 0.0 else Vector2(size.y, size.x)
	return Rect2(pos - s * 0.5, s)


static func generate(rng: RandomNumberGenerator, count: int, bounds: Rect2,
		walls: Array[Rect2], keep_clear: Array[Vector3], sizes: Array[Vector2],
		min_gap: float = 1.5, max_attempts: int = 2000) -> Array[Dictionary]:
	var out: Array[Dictionary] = []
	var placed: Array[Rect2] = []
	var clear_rects: Array[Rect2] = []
	for c: Vector3 in keep_clear:
		clear_rects.append(Rect2(Vector2(c.x, c.z) - Vector2(1, 1), Vector2(2, 2)))
	var inner: Rect2 = bounds.grow(-1.0)
	var attempts: int = 0
	while out.size() < count and attempts < max_attempts:
		attempts += 1
		var size: Vector2 = sizes[rng.randi_range(0, sizes.size() - 1)]
		var rot: float = 0.0 if rng.randf() < 0.5 else PI * 0.5
		var pos := Vector2(
			rng.randf_range(inner.position.x + 1.0, inner.end.x - 1.0),
			rng.randf_range(inner.position.y + 1.0, inner.end.y - 1.0))
		var r: Rect2 = _footprint(pos, size, rot)
		if not inner.encloses(r):
			continue
		var ok: bool = true
		for w: Rect2 in walls:
			if overlaps(r, w, min_gap):
				ok = false
				break
		if ok:
			for p: Rect2 in placed:
				if overlaps(r, p, min_gap):
					ok = false
					break
		if ok:
			for c: Rect2 in clear_rects:
				if overlaps(r, c, min_gap):
					ok = false
					break
		if not ok:
			continue
		placed.append(r)
		out.append({"pos": pos, "size": size, "rot": rot})
	return out
