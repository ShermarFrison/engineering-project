## Shared setup for scene tests that need the arena with a baked, ready navmesh.
class_name NavArenaHelper
extends RefCounted

const ARENA := preload("res://scenes/arena/arena.tscn")
const ROBOT := preload("res://scenes/robots/robot.tscn")


## Loads the arena through the suite's scene runner (auto-freed with the test),
## bakes the walls-only navmesh and waits up to 30 physics ticks for path
## queries to work. Returns [runner, arena].
static func ready_arena(suite: PhysicsTestSuite) -> Array:
	var runner: GdUnitSceneRunner = suite.scene_runner(suite.auto_free(ARENA.instantiate()))
	var arena: Arena = runner.scene()
	arena.nav.bake(arena.static_root)
	var ok: bool = await suite.ticks_until(arena.nav.is_ready, 30)
	suite.assert_bool(ok).append_failure_message("navmesh never became ready within 30 ticks").is_true()
	return [runner, arena]


static func spawn_robot(arena: Arena, pos: Vector3, role: String) -> DriveBody:
	var r: DriveBody = ROBOT.instantiate()
	r.role = role
	r.position = pos + Vector3(0, 0.3, 0)
	arena.robots_root.add_child(r)
	return r


static func los_walls(arena: Arena) -> Callable:
	return func(a: Vector3, b: Vector3) -> bool:
		var q := PhysicsRayQueryParameters3D.create(a + Vector3(0, 0.3, 0), b + Vector3(0, 0.3, 0), 1)
		return arena.get_world_3d().direct_space_state.intersect_ray(q).is_empty()
