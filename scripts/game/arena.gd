## Hand-built arena. Walls live under Static; Props and Robots are filled at runtime.
## build_maze() adds seeded interior walls under Static as well, so wall_rects() and
## the navmesh bake see them like any hand-built wall.
class_name Arena
extends Node3D

const WALL_HEIGHT: float = 1.0
const MAZE_PREFIX: String = "Maze"

@onready var static_root: Node3D = $Static
@onready var props_root: Node3D = $Props
@onready var robots_root: Node3D = $Robots
@onready var nav: NavService = $NavService

var _maze_bodies: Array[Node] = []
var _maze_size: Vector2i = Vector2i.ZERO  # rows, cols; zero = no maze, spawns are not snapped


func _ready() -> void:
	_update_probes()


func _update_probes() -> void:
	var hs: Array[Vector3] = hider_spawn_points()
	nav.set_probe_points(seeker_spawn_point(), hs[0])


func bounds() -> Rect2:
	return Rect2(-16.0, -16.0, 32.0, 32.0)


func seeker_spawn_point() -> Vector3:
	return _snap(($SeekerSpawn as Marker3D).global_position)


func hider_spawn_points() -> Array[Vector3]:
	var out: Array[Vector3] = []
	for m: Node in $HiderSpawns.get_children():
		out.append(_snap((m as Marker3D).global_position))
	return out


## Spawn markers sit on whole metres, which are maze wall lines; move them to the cell centre.
func _snap(p: Vector3) -> Vector3:
	if _maze_size == Vector2i.ZERO:
		return p
	var b: Rect2 = bounds()
	var cell := Vector2(b.size.x / _maze_size.y, b.size.y / _maze_size.x)
	var c: int = clampi(floori((p.x - b.position.x) / cell.x), 0, _maze_size.y - 1)
	var r: int = clampi(floori((p.z - b.position.y) / cell.y), 0, _maze_size.x - 1)
	return Vector3(b.position.x + (c + 0.5) * cell.x, p.y, b.position.y + (r + 0.5) * cell.y)


## Replaces any previous maze with a new seeded one covering bounds().
func build_maze(rng: RandomNumberGenerator, rows: int, cols: int, loop_prob: float, thickness: float) -> void:
	clear_maze()
	var b: Rect2 = bounds()
	var walls: PackedByteArray = MazeGenerator.generate(rng, rows, cols, loop_prob)
	var rects: Array[Rect2] = MazeGenerator.interior_wall_rects(
			walls, rows, cols, b.position, b.size.x / cols, thickness)
	var mat: Material = ($Static/OuterN/Mesh as MeshInstance3D).get_surface_override_material(0)
	for i: int in rects.size():
		var r: Rect2 = rects[i]
		var size := Vector3(r.size.x, WALL_HEIGHT, r.size.y)
		var body := StaticBody3D.new()
		body.name = "%s%d" % [MAZE_PREFIX, i]
		body.collision_mask = 0
		body.position = Vector3(r.get_center().x, WALL_HEIGHT * 0.5, r.get_center().y)
		var shape := BoxShape3D.new()
		shape.size = size
		var cs := CollisionShape3D.new()
		cs.shape = shape
		body.add_child(cs)
		var mesh := BoxMesh.new()
		mesh.size = size
		var mi := MeshInstance3D.new()
		mi.mesh = mesh
		mi.set_surface_override_material(0, mat)
		body.add_child(mi)
		static_root.add_child(body)
		_maze_bodies.append(body)
	_maze_size = Vector2i(rows, cols)
	_update_probes()


## Removed from the tree at once (not just queued) so a bake right after no longer sees them.
func clear_maze() -> void:
	for body: Node in _maze_bodies:
		static_root.remove_child(body)
		body.queue_free()
	_maze_bodies.clear()
	_maze_size = Vector2i.ZERO
	_update_probes()


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
