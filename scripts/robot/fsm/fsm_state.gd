## One state of a finite state machine. Reads/writes only the Blackboard.
class_name FSMState
extends RefCounted

var name: String = ""
## Why the next transition happens; set just before returning a state name from update().
## FSM logs it with the transition and clears it.
var reason: String = ""


func enter(_bb: Blackboard) -> void:
	pass


## Return "" to stay, or the name of the next state (set `reason` first).
func update(_bb: Blackboard, _dt: float) -> String:
	return ""


func exit(_bb: Blackboard) -> void:
	pass
