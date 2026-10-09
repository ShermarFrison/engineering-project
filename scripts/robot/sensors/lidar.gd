## 2D lidar: a horizontal fan of raycasts. Unhit rays report max_range.
class_name Lidar
extends SensorBase

@export var ray_count: int = 36
@export var max_range: float = 8.0
@export var collision_mask: int = 7


func sense(bb: Blackboard) -> void:
	bb.lidar_enabled = enabled
	bb.lidar_ranges.clear()
	bb.lidar_angles.clear()
	bb.lidar_max_range = max_range
	if not enabled:
		return
	var origin: Vector3 = global_position
	var space := _space()
	var exclude: Array[RID] = [body.get_rid()]  # the origin is inside our own mast
	for i: int in ray_count:
		var angle: float = wrapf(TAU * i / ray_count, -PI, PI)
		var dir: Vector3 = bb.local_to_world_dir(angle)
		var q := PhysicsRayQueryParameters3D.create(origin, origin + dir * max_range, collision_mask, exclude)
		var hit: Dictionary = space.intersect_ray(q)
		var r: float = max_range
		if not hit.is_empty():
			r = minf(max_range, origin.distance_to(hit["position"]) + _noise(bb))
		bb.lidar_ranges.append(r)
		bb.lidar_angles.append(angle)
