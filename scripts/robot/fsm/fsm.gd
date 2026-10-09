## Minimal FSM with a transition log for the HUD and CSV.
class_name FSM
extends RefCounted

var states: Dictionary = {}  # name -> FSMState
var current: FSMState = null
var transitions: Array[Dictionary] = []
var transition_counts: Dictionary = {}


func add(state: FSMState) -> void:
	states[state.name] = state


func start(name: String, bb: Blackboard) -> void:
	current = states[name]
	bb.state_name = name
	current.enter(bb)


func current_name() -> String:
	return current.name if current else ""


func update(bb: Blackboard, dt: float) -> void:
	if current == null:
		return
	var state: FSMState = current
	var next: String = state.update(bb, dt)
	var why: String = state.reason
	state.reason = ""
	if next != "" and next != state.name:
		_transition(next, bb, why)


## Test seam (e.g. Task 16 forces the seeker into Chase).
func force(name: String, bb: Blackboard) -> void:
	if current == null or name == current.name:
		return
	_transition(name, bb, "forced")


func _transition(to: String, bb: Blackboard, why: String) -> void:
	assert(states.has(to), "unknown state %s" % to)
	var from: String = current.name
	current.exit(bb)
	transitions.append({"time": bb.time, "from": from, "to": to, "reason": why})
	var key: String = "%s->%s" % [from, to]
	transition_counts[key] = int(transition_counts.get(key, 0)) + 1
	current = states[to]
	bb.state_name = to
	current.enter(bb)
