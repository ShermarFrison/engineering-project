## Single-state FSM payload for tests and debugging: drive to a fixed target.
class_name GoToState
extends FSMState

var target: Vector3 = Vector3.ZERO


func _init(t: Vector3 = Vector3.ZERO) -> void:
	name = "GoTo"
	target = t


func enter(bb: Blackboard) -> void:
	bb.goal_pos = target
	bb.goal_active = true
