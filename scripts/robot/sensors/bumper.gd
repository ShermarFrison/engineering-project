## Front-arc contact sensor fed by DriveBody.contacts.
class_name Bumper
extends SensorBase

@export var front_half_deg: float = 70.0


func sense(bb: Blackboard) -> void:
	bb.bumper_enabled = enabled
	bb.bumper_hit = false
	bb.bumper_normal = Vector3.ZERO
	if not enabled:
		return
	var half: float = deg_to_rad(front_half_deg)
	for c: Dictionary in body.contacts:
		var angle: float = bb.world_to_local_angle(c["position"])
		if absf(angle) <= half:
			bb.bumper_hit = true
			bb.bumper_normal = c["normal"]
			return
