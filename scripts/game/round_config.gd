## Everything that varies between rounds. Saved as .tres for experiments.
class_name RoundConfig
extends Resource

@export var variant_name: String = "default"
@warning_ignore("shadowed_global_identifier")  # name fixed by the experiment config interface
@export var seed: int = 0
@export var hider_count: int = 2
@export var head_start: float = 15.0
@export var round_time: float = 120.0
@export var tag_range: float = 1.5
@export var prop_count: int = 12
@export var seeker_sensors: PackedStringArray = PackedStringArray(["lidar", "vision", "ultrasonic", "bumper"])
@export var hider_sensors: PackedStringArray = PackedStringArray(["lidar", "vision", "ultrasonic", "bumper"])
@export var hider_strategy: String = "static"  # "static" | "dynamic"
@export var noise_std: float = 0.0


func has_sensor(role: String, sensor: String) -> bool:
	var list: PackedStringArray = seeker_sensors if role == "seeker" else hider_sensors
	return list.has(sensor)


func sensors_label(role: String) -> String:
	var list: PackedStringArray = seeker_sensors if role == "seeker" else hider_sensors
	return "+".join(list)
