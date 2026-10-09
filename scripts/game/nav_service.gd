## Bakes a walls-only navmesh from a given root and answers path queries.
## Readiness must be polled: the NavigationServer needs a few physics frames
## after the region is added before queries return anything (see NOTES).
class_name NavService
extends Node

@export var agent_radius: float = 0.35
@export var cell_size: float = 0.25

var _region: NavigationRegion3D
var _probe_a: Vector3
var _probe_b: Vector3
var _ready_flag: bool = false


func bake(source_root: Node3D) -> void:
	var nm := NavigationMesh.new()
	nm.agent_radius = agent_radius
	nm.cell_size = cell_size
	nm.cell_height = 0.25
	nm.agent_height = 0.5
	nm.geometry_parsed_geometry_type = NavigationMesh.PARSED_GEOMETRY_STATIC_COLLIDERS
	nm.geometry_collision_mask = 1  # walls + floor only
	var src := NavigationMeshSourceGeometryData3D.new()
	NavigationServer3D.parse_source_geometry_data(nm, src, source_root)
	NavigationServer3D.bake_from_source_geometry_data(nm, src)
	if _region:
		_region.queue_free()
	_region = NavigationRegion3D.new()
	_region.navigation_mesh = nm
	add_child(_region)
	var map: RID = get_viewport().world_3d.navigation_map
	NavigationServer3D.map_set_use_async_iterations(map, false)
	_ready_flag = false


func set_probe_points(a: Vector3, b: Vector3) -> void:
	_probe_a = a
	_probe_b = b


func is_ready() -> bool:
	if _ready_flag:
		return true
	if _region == null:
		return false
	var path: PackedVector3Array = find_path(_probe_a, _probe_b)
	_ready_flag = path.size() >= 2
	return _ready_flag


## Named find_path, not get_path: a Node subclass may not override Node.get_path().
func find_path(from: Vector3, to: Vector3) -> PackedVector3Array:
	var map: RID = get_viewport().world_3d.navigation_map
	return NavigationServer3D.map_get_path(map, from, to, true)
