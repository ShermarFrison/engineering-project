## Camera-like detector: limited FOV and range, confirmed by a line-of-sight ray.
## noise_std (metres) perturbs the measured distance, bearing_noise_std (radians) the
## bearing; the reported position is rebuilt from the noisy pair, so consumers see the noise.
class_name VisionCone
extends SensorBase

@export var fov_deg: float = 90.0
@export var max_range: float = 10.0
@export var los_mask: int = 7
@export var los_target_height: float = 0.125  # centre of the target's mast above its body origin
@export var bearing_noise_std: float = 0.0  # radians


func sense(bb: Blackboard) -> void:
	bb.vision_enabled = enabled
	bb.seen_robots.clear()
	if not enabled:
		return
	var eye: Vector3 = global_position
	var half_fov: float = deg_to_rad(fov_deg) * 0.5
	var space := _space()
	var exclude: Array[RID] = [body.get_rid()]  # never let our own mast block the ray
	for node: Node in get_tree().get_nodes_in_group("robots"):
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
		var seen_bearing: float = bearing + _gauss(bb, bearing_noise_std)
		var seen_dist: float = clampf(dist + _noise(bb), 0.0, max_range)
		var seen_pos: Vector3 = other.global_position
		if noise_std > 0.0 or bearing_noise_std > 0.0:
			seen_pos = bb.position + bb.local_to_world_dir(seen_bearing) * seen_dist
			seen_pos.y = other.global_position.y  # noise is planar; keep the target's height
		var rec := {
			"id": id,
			"role": other.role,
			"position": seen_pos,
			"bearing": seen_bearing,
			"distance": seen_dist,
			"time": bb.time,
		}
		bb.seen_robots.append(rec)
		bb.last_known[id] = {"position": seen_pos, "time": bb.time, "role": other.role}
		if other.role == "seeker":
			bb.seeker_last_seen_pos = seen_pos
			bb.seeker_last_seen_time = bb.time
