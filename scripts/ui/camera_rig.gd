## Orbit camera: right-drag to orbit, wheel to zoom, optional follow target.
class_name CameraRig
extends Node3D

@export var distance: float = 25.0
@export var pitch: float = -55.0
@export var yaw_deg: float = 0.0

var _target: Node3D = null
var _following: bool = false  # a freed target is neither valid nor reliably == null
var _dragging: bool = false
@onready var _cam: Camera3D = $Camera3D


func follow(body: Node3D) -> void:
	_target = body
	_following = true
	distance = minf(distance, 10.0)


func free_camera() -> void:
	_target = null
	_following = false
	distance = 25.0


func is_following() -> bool:
	return _following


func _unhandled_input(event: InputEvent) -> void:
	if event is InputEventMouseButton:
		var mb := event as InputEventMouseButton
		if mb.button_index == MOUSE_BUTTON_RIGHT:
			_dragging = mb.pressed
		elif mb.button_index == MOUSE_BUTTON_WHEEL_UP:
			distance = maxf(3.0, distance - 1.5)
		elif mb.button_index == MOUSE_BUTTON_WHEEL_DOWN:
			distance = minf(60.0, distance + 1.5)
	elif event is InputEventMouseMotion and _dragging:
		var mm := event as InputEventMouseMotion
		yaw_deg -= mm.relative.x * 0.3
		pitch = clampf(pitch - mm.relative.y * 0.3, -89.0, -10.0)


func _process(_dt: float) -> void:
	if _following and not is_instance_valid(_target):
		free_camera()  # the followed robot was freed (round restarted)
	if _following:
		global_position = global_position.lerp(_target.global_position, 0.15)
	else:
		global_position = global_position.lerp(Vector3.ZERO, 0.1)
	rotation_degrees = Vector3(0.0, yaw_deg, 0.0)
	# Place the camera `distance` away along a ray tilted by `pitch` below horizontal, looking at the rig origin.
	var pivot := Transform3D.IDENTITY.rotated(Vector3.RIGHT, deg_to_rad(pitch))
	_cam.transform = pivot * Transform3D(Basis.IDENTITY, Vector3(0, 0, distance))
	_cam.look_at(global_position, Vector3.UP)
