extends PhysicsTestSuite


func _rig() -> CameraRig:
	var rig := CameraRig.new()
	var cam := Camera3D.new()
	cam.name = "Camera3D"
	rig.add_child(cam)
	var runner := scene_runner(auto_free(rig))
	assert_object(runner.scene()).is_same(rig)
	return rig


func test_follow_falls_back_to_free_camera_when_target_is_freed() -> void:
	var rig := _rig()
	var target := Node3D.new()
	rig.get_parent().add_child(target)
	target.global_position = Vector3(5, 0, 5)
	rig.follow(target)
	await get_tree().process_frame
	assert_bool(rig.is_following()).is_true()
	target.free()
	await get_tree().process_frame
	await get_tree().process_frame
	assert_bool(rig.is_following()).is_false()
	assert_float(rig.distance).is_equal(25.0)
	# Not frozen: the free camera drifts back toward the arena centre.
	var d0: float = rig.global_position.length()
	for i: int in 5:
		await get_tree().process_frame
	assert_float(rig.global_position.length()).is_less(d0)
