## Reactive heading selection: score every lidar direction by alignment with
## the target minus a proximity penalty, pick the best. Pure function.
class_name Steering
extends RefCounted


static func pick_heading(target_angle: float, ranges: PackedFloat32Array, angles: PackedFloat32Array,
		max_range: float, avoid_dist: float = 1.5, window: float = deg_to_rad(20.0)) -> float:
	var n: int = ranges.size()
	if n == 0:
		return target_angle
	var best_a: float = target_angle
	var best_s: float = -INF
	for i in n:
		var a: float = angles[i]
		var clearance: float = INF
		for j in n:
			if absf(wrapf(angles[j] - a, -PI, PI)) <= window + 1e-6:
				clearance = minf(clearance, ranges[j])
		if clearance >= max_range - 1e-3:
			clearance = INF
		var penalty: float = 0.0
		if clearance < avoid_dist:
			penalty = 2.0 * (avoid_dist - clearance) / avoid_dist
		var score: float = cos(wrapf(a - target_angle, -PI, PI)) - penalty
		if score > best_s + 1e-9:
			best_s = score
			best_a = a
	# If the target direction itself is clear, use it exactly (avoid quantising to ray angles).
	var target_clear: float = INF
	for j in n:
		if absf(wrapf(angles[j] - target_angle, -PI, PI)) <= window + 1e-6:
			target_clear = minf(target_clear, ranges[j])
	if target_clear >= avoid_dist:
		return target_angle
	return best_a


static func speed_scale(front_clearance: float, slow_dist: float = 2.0, min_scale: float = 0.2) -> float:
	if is_inf(front_clearance):
		return 1.0
	return clampf(front_clearance / slow_dist, min_scale, 1.0)
