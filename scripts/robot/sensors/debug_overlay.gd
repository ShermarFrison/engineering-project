## Draws lidar rays and the vision cone with an ImmediateMesh. Toggle with
## DebugOverlay.visible_globally (F1, handled by main.gd). Draws in world space.
class_name DebugOverlay
extends MeshInstance3D

static var visible_globally: bool = false

var _im := ImmediateMesh.new()


func _ready() -> void:
	mesh = _im
	var mat := StandardMaterial3D.new()
	mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	mat.vertex_color_use_as_albedo = true
	mat.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	material_override = mat
	top_level = true  # we draw in world space


func draw(bb: Blackboard, vision: VisionCone) -> void:
	_im.clear_surfaces()
	if not visible_globally:
		return
	global_transform = Transform3D.IDENTITY
	var origin: Vector3 = bb.position + Vector3(0, 0.15, 0)
	if bb.lidar_enabled and bb.lidar_ranges.size() > 0:
		_im.surface_begin(Mesh.PRIMITIVE_LINES)
		for i: int in bb.lidar_ranges.size():
			var r: float = bb.lidar_ranges[i]
			var hit: bool = r < bb.lidar_max_range - 1e-3
			_im.surface_set_color(Color(1, 0.3, 0.2, 0.9) if hit else Color(0.4, 0.9, 0.4, 0.25))
			_im.surface_add_vertex(origin)
			_im.surface_add_vertex(origin + bb.local_to_world_dir(bb.lidar_angles[i]) * r)
		_im.surface_end()
	if bb.vision_enabled and vision != null:
		var half: float = deg_to_rad(vision.fov_deg) * 0.5
		var col := Color(0.3, 0.6, 1.0, 0.8) if bb.seen_robots.is_empty() else Color(1.0, 0.9, 0.2, 0.9)
		_im.surface_begin(Mesh.PRIMITIVE_LINE_STRIP)
		_im.surface_set_color(col)
		_im.surface_add_vertex(origin)
		for k: int in 17:
			var a: float = -half + 2.0 * half * k / 16.0
			_im.surface_add_vertex(origin + bb.local_to_world_dir(a) * vision.max_range)
		_im.surface_add_vertex(origin)
		_im.surface_end()
	if bb.goal_active:
		_im.surface_begin(Mesh.PRIMITIVE_LINES)
		_im.surface_set_color(Color(1, 1, 1, 0.8))
		_im.surface_add_vertex(origin)
		_im.surface_add_vertex(bb.goal_pos + Vector3(0, 0.15, 0))
		_im.surface_end()
