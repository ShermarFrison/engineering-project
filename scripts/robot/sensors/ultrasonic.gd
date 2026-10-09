## Three short-range proximity cones (front, left, right). Each cone is a few
## rays; the minimum hit distance is reported, max_range when clear.
class_name Ultrasonic
extends SensorBase

@export var max_range: float = 1.2
@export var cone_half_deg: float = 15.0
@export var rays_per_cone: int = 3
@export var directions_deg: PackedFloat32Array = PackedFloat32Array([0.0, 35.0, -35.0])
@export var collision_mask: int = 7


func sense(bb: Blackboard) -> void:
	bb.ultrasonic_enabled = enabled
	bb.ultrasonic = PackedFloat32Array([max_range, max_range, max_range])
	if not enabled:
		return
	var origin: Vector3 = global_position
	var space := _space()
	var exclude: Array[RID] = [body.get_rid()]  # the origin sits on our own mast's front face
	for c: int in directions_deg.size():
		var center: float = deg_to_rad(directions_deg[c])
		var best: float = max_range
		for k: int in rays_per_cone:
			var t: float = 0.0
			if rays_per_cone > 1:
				t = -1.0 + 2.0 * k / (rays_per_cone - 1)
			var angle: float = center + t * deg_to_rad(cone_half_deg)
			var dir: Vector3 = bb.local_to_world_dir(angle)
			var q := PhysicsRayQueryParameters3D.create(origin, origin + dir * max_range, collision_mask, exclude)
			var hit: Dictionary = space.intersect_ray(q)
			if not hit.is_empty():
				best = minf(best, origin.distance_to(hit["position"]) + _noise(bb))
		bb.ultrasonic[c] = clampf(best, 0.0, max_range)
