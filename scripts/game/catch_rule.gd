## The single predicate deciding whether a hider is caught.
class_name CatchRule
extends RefCounted


static func is_catch(distance: float, tag_range: float, seeker_state: String, los_clear: bool) -> bool:
	if distance >= tag_range:  # spec: catch when distance < tag range
		return false
	if not los_clear:
		return false
	return seeker_state == "Chase" or seeker_state == "Catch"
