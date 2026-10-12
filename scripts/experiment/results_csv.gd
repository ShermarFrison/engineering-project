## Flattens a GameManager results Dictionary into CSV rows.
class_name ResultsCsv
extends RefCounted

const MAX_HIDERS: int = 3
const HIDER_FIELDS: PackedStringArray = ["caught", "catch_time", "distance", "collisions", "stuck", "transitions"]


static func header() -> PackedStringArray:
	var h: PackedStringArray = ["variant", "seed", "hider_strategy", "seeker_sensors", "hider_sensors",
		"hiders_total", "hiders_caught", "duration",
		"seeker_distance", "seeker_collisions", "seeker_stuck", "seeker_transitions"]
	for i: int in MAX_HIDERS:
		for f: String in HIDER_FIELDS:
			h.append("hider%d_%s" % [i, f])
	return h


static func _f(x: float) -> String:
	return "%.3f" % x


static func row(res: Dictionary) -> PackedStringArray:
	var s: Dictionary = res["seeker"]
	var r: PackedStringArray = [
		str(res["variant"]), str(res["seed"]), str(res["hider_strategy"]),
		str(res["seeker_sensors"]), str(res["hider_sensors"]),
		str(res["hiders_total"]), str(res["hiders_caught"]), _f(res["duration"]),
		_f(s["distance"]), str(s["collisions"]), str(s["stuck_events"]), str(s["transitions"].size()),
	]
	var hiders: Array = res["hiders"]
	for i: int in MAX_HIDERS:
		if i < hiders.size():
			var h: Dictionary = hiders[i]
			r.append("1" if h["caught"] else "0")
			r.append(_f(h["catch_time"]))
			r.append(_f(h["distance"]))
			r.append(str(h["collisions"]))
			r.append(str(h["stuck_events"]))
			r.append(str(h["transitions"].size()))
		else:
			for f: String in HIDER_FIELDS:
				r.append("")
	return r


static func transitions_header() -> PackedStringArray:
	return PackedStringArray(["variant", "seed", "robot", "time", "from", "to", "reason"])


static func transitions_rows(res: Dictionary) -> Array[PackedStringArray]:
	var out: Array[PackedStringArray] = []
	for t: Dictionary in res["seeker"]["transitions"]:
		out.append(PackedStringArray([str(res["variant"]), str(res["seed"]), "seeker", _f(t["time"]),
			str(t["from"]), str(t["to"]), str(t["reason"])]))
	var hiders: Array = res["hiders"]
	for i: int in hiders.size():
		for t: Dictionary in hiders[i]["transitions"]:
			out.append(PackedStringArray([str(res["variant"]), str(res["seed"]), "hider%d" % i, _f(t["time"]),
				str(t["from"]), str(t["to"]), str(t["reason"])]))
	return out
