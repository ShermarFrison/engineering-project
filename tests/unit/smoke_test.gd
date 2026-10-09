extends GdUnitTestSuite


func test_project_name() -> void:
	assert_str(ProjectSettings.get_setting("application/config/name")).is_equal("Robot Hide and Seek")
