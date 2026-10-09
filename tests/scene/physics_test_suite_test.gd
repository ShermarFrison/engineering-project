extends PhysicsTestSuite


func test_ticks_advance_exactly_n_physics_frames() -> void:
	var start: int = Engine.get_physics_frames()
	await ticks(60)
	assert_int(Engine.get_physics_frames() - start).is_equal(60)


func test_ticks_until_returns_true_when_predicate_holds() -> void:
	var start: int = Engine.get_physics_frames()
	var ok: bool = await ticks_until(func() -> bool: return Engine.get_physics_frames() - start >= 10, 100)
	assert_bool(ok).is_true()
	assert_int(Engine.get_physics_frames() - start).is_between(10, 11)


func test_ticks_until_gives_up_after_budget() -> void:
	var start: int = Engine.get_physics_frames()
	var ok: bool = await ticks_until(func() -> bool: return false, 20)
	assert_bool(ok).is_false()
	assert_int(Engine.get_physics_frames() - start).is_equal(20)


func test_scene_runner_with_auto_free_and_ticks() -> void:
	var root := Node3D.new()
	var body := RigidBody3D.new()
	var cs := CollisionShape3D.new()
	cs.shape = BoxShape3D.new()
	body.add_child(cs)
	root.add_child(body)
	var runner := scene_runner(auto_free(root))
	await ticks(30)
	assert_float(body.global_position.y).is_less(0.0)  # no floor: it falls under gravity
	assert_object(runner.scene()).is_same(root)
