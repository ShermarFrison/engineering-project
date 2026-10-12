## Hand-built arena. Walls live under Static; Props and Robots are filled at runtime.
class_name Arena
extends Node3D

@onready var static_root: Node3D = $Static
@onready var props_root: Node3D = $Props
@onready var robots_root: Node3D = $Robots
@onready var nav: NavService = $NavService


func _ready() -> void:
	var hs: Array[Vector3] = hider_spawn_points()
	nav.set_probe_points(seeker_spawn_point(), hs[0])


func bounds() -> Rect2:
	return Rect2(-15.0, -15.0, 30.0, 30.0)


func seeker_spawn_point() -> Vector3:
	return ($SeekerSpawn as Marker3D).global_position


func hider_spawn_points() -> Array[Vector3]:
	var out: Array[Vector3] = []
	for m: Node in $HiderSpawns.get_children():
		out.append((m as Marker3D).global_position)
	return out


## XZ footprints of all wall boxes (axis-aligned; walls are never rotated).
func wall_rects() -> Array[Rect2]:
	var out: Array[Rect2] = []
	for body: Node in static_root.get_children():
		if body.name == "Floor":
			continue
		for shape_node: Node in body.get_children():
			var cs := shape_node as CollisionShape3D
			if cs == null or not (cs.shape is BoxShape3D):
				continue
			var size: Vector3 = (cs.shape as BoxShape3D).size
			var c: Vector3 = cs.global_position
			out.append(Rect2(c.x - size.x * 0.5, c.z - size.z * 0.5, size.x, size.z))
	return out
