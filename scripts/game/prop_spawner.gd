## Instantiates crate scenes from a PropLayout result.
class_name PropSpawner
extends Node3D

const CRATE := preload("res://scenes/props/crate.tscn")
const SIZES: Array[Vector2] = [Vector2(1.0, 1.0), Vector2(1.5, 1.0), Vector2(2.0, 1.0), Vector2(1.2, 1.2)]
const HEIGHT: float = 1.0


func clear(parent: Node3D) -> void:
	for c in parent.get_children():
		parent.remove_child(c)
		c.queue_free()


func spawn(layout: Array[Dictionary], parent: Node3D) -> void:
	for i in layout.size():
		var p: Dictionary = layout[i]
		var crate: StaticBody3D = CRATE.instantiate()
		crate.name = "Crate%d" % i
		parent.add_child(crate)
		crate.scale = Vector3(p["size"].x, HEIGHT, p["size"].y)
		crate.rotation.y = p["rot"]
		crate.position = Vector3(p["pos"].x, HEIGHT * 0.5, p["pos"].y)
