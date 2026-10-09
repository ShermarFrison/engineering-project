## Base class for scene tests. Waits are counted in physics ticks (60 per
## simulated second), never in process frames: headless process frames are
## not throttled, so simulate_frames(n) is not n/60 seconds.
class_name PhysicsTestSuite
extends GdUnitTestSuite


## Await exactly n physics ticks (60 ticks = 1 simulated second).
func ticks(n: int) -> void:
	for i in n:
		await get_tree().physics_frame


## Await physics ticks until pred.call() is true or max_ticks have passed.
## Returns true if the predicate became true within the budget.
func ticks_until(pred: Callable, max_ticks: int) -> bool:
	for i in max_ticks:
		if pred.call():
			return true
		await get_tree().physics_frame
	return pred.call()
