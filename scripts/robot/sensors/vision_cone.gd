## Camera-like detector: limited FOV and range, confirmed by a line-of-sight ray.
class_name VisionCone
extends SensorBase

@export var fov_deg: float = 90.0
@export var max_range: float = 10.0
@export var los_mask: int = 7
@export var los_target_height: float = 0.25  # centre of the target's mast above its body origin


func sense(bb: Blackboard) -> void:
	bb.vision_enabled = enabled
	bb.seen_robots.clear()
	if not enabled:
		return
	var eye: Vector3 = global_position
	var half_fov: float = deg_to_rad(fov_deg) * 0.5
	var space := _space()
	var exclude: Array[RID] = [body.get_rid()]  # never let our own mast block the ray
	for node in get_tree().get_nodes_in_group("robots"):
		var other := node as DriveBody
		if other == null or other == body:
			continue
		var target: Vector3 = other.global_position + Vector3(0, los_target_height, 0)
		var flat: Vector3 = other.global_position - bb.position  # centre to centre, like the tag range
		flat.y = 0.0
		var dist: float = flat.length()
		if dist > max_range:
			continue
		var bearing: float = bb.world_to_local_angle(other.global_position)
		if absf(bearing) > half_fov:
			continue
		var q := PhysicsRayQueryParameters3D.create(eye, target, los_mask, exclude)
		var hit: Dictionary = space.intersect_ray(q)
		if hit.is_empty() or hit["collider_id"] != other.get_instance_id():
			continue
		var id: int = other.get_instance_id()
		var rec := {
			"id": id,
			"role": other.role,
			"position": other.global_position,
			"bearing": bearing + _noise(bb),
			"distance": dist + _noise(bb),
			"time": bb.time,
		}
		bb.seen_robots.append(rec)
		bb.last_known[id] = {"position": other.global_position, "time": bb.time, "role": other.role}
		if other.role == "seeker":
			bb.seeker_last_seen_pos = other.global_position
			bb.seeker_last_seen_time = bb.time
