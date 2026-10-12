extends PhysicsTestSuite

const ARENA := preload("res://scenes/arena/arena.tscn")


func test_spawn_places_crates_that_block_rays() -> void:
	var runner := scene_runner(auto_free(ARENA.instantiate()))
	var arena: Arena = runner.scene()
	var spawner := PropSpawner.new()
	arena.add_child(spawner)
	var rng := RandomNumberGenerator.new()
	rng.seed = 11
	# Build the typed keep-clear list with append_array: `[a] + typed_array` is an
	# untyped Array and fails at runtime when passed as Array[Vector3].
	var keep: Array[Vector3] = [arena.seeker_spawn_point()]
	keep.append_array(arena.hider_spawn_points())
	var layout := PropLayout.generate(rng, 12, arena.bounds(), arena.wall_rects(), keep, PropSpawner.SIZES)
	spawner.spawn(layout, arena.props_root)
	await ticks(2)
	assert_int(arena.props_root.get_child_count()).is_equal(layout.size())
	var space := arena.get_world_3d().direct_space_state
	var p: Dictionary = layout[0]
	var from := Vector3(p["pos"].x, 0.5, p["pos"].y) + Vector3(5, 0, 0)
	var q := PhysicsRayQueryParameters3D.create(from, Vector3(p["pos"].x, 0.5, p["pos"].y), 2)
	var hit: Dictionary = space.intersect_ray(q)
	assert_bool(hit.is_empty()).is_false()
	assert_bool((instance_from_id(hit["collider_id"]) as Node).is_in_group("props")).is_true()
	spawner.clear(arena.props_root)
	await ticks(1)
	assert_int(arena.props_root.get_child_count()).is_equal(0)
