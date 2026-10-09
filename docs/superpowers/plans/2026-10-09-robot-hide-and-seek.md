# Robot Hide and Seek Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A Godot 4.7.2 game where one autonomous differential-drive seeker robot hunts 2–3 autonomous hider robots in a hand-built arena with randomized props, plus a headless batch runner that writes per-round CSV results for sensor-ablation and catch-rate experiments.

**Architecture:** Each robot is a layered stack: sensor nodes (lidar, vision cone, ultrasonic, bumper) write into a `Blackboard`; a role-specific finite state machine reads only the blackboard and writes a goal; a `Motion` controller turns the goal into (v, ω) via navmesh path following blended with reactive lidar avoidance; a `DriveBody` (RigidBody3D) applies the two-wheel unicycle model in `_integrate_forces`. A `GameManager` owns the round lifecycle, the single seeded RNG, and catch detection. A `BatchRunner` loops variants × seeds and writes CSV.

**Tech Stack:** Godot 4.7.2 (bundled `./Godot_v4.7.2-stable_linux.x86_64`), GDScript, Jolt Physics, NavigationServer3D, gdUnit4 6.2.2 for tests, Python 3 + matplotlib for offline analysis.

**Spec:** `docs/superpowers/specs/2026-10-09-robot-hide-and-seek-design.md`

## Global Constraints

- Engine: Godot `4.7.2.stable.official`, run as `./Godot_v4.7.2-stable_linux.x86_64` from the project root. Physics engine `Jolt Physics`, renderer `Forward Plus` (already in `project.godot`).
- Language: GDScript only. Static typing on every declaration (`var x: int`, `-> void`).
- Dependencies: gdUnit4 6.2.2 under `addons/gdUnit4`. No other addons. No external 3D assets; geometry from `BoxMesh`, `CylinderMesh`, `CSGBox3D`.
- Scenes are authored as text `.tscn` / `.tres` files. The editor (via computer-use-vnc) is used only to inspect, run, and screenshot.
- All randomness goes through one `RandomNumberGenerator` owned by `GameManager` and passed down. Never call `randf()`/`randi()` globally.
- All robot logic runs in `_physics_process` (fixed 60 Hz ticks); nothing behaviour-relevant in `_process`.
- Forward axis is `-Z` (Godot convention). Yaw is rotation about `+Y`; positive yaw / positive bearing = turn left (counter-clockwise from above).
- Collision layers: 1 = walls + floor, 2 = props, 3 = robots.
- Sensors never read other robots' blackboards or scene state directly except through physics queries; vision candidate lookup uses the `robots` group followed by a line-of-sight raycast.
- Tests: `./run_tests.sh` must exit 0 before every commit. It runs `--import` then the gdUnit4 CLI on `res://tests`.
- Commit after every task with the message style `feat|test|chore: <summary>` and the trailer `Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>`.
- Verified engine facts to rely on (see `NOTES/godot-headless-gotchas.md`): chassis needs `friction = 0.0` material; navmesh must be baked from the arena root via `parse_source_geometry_data`; path queries need ~3 physics frames after the region is added with synchronous map iterations.

## Review Focus

1. **A hider is inside tag range but behind a crate.** Expected: not caught (LOS blocked by props). Pinned in Task 16 `test_catch_rule_blocked_los`.
2. **Seeker has vision disabled and bumps into a hider by chance.** Expected: not a catch unless the seeker is in Chase/Catch, which requires a lidar blob detection. Pinned in Task 16 `test_catch_requires_chase_state` and `test_blind_seeker_touching_hider_does_not_catch`, and Task 13 `test_patrol_to_investigate_on_blob`.
3. **Prop spawner given a seed where random positions keep colliding.** Expected: it gives up after a bounded number of attempts and places fewer props rather than hanging. Pinned in Task 5 `test_spawner_terminates_on_crowded_arena`.
4. **Round time runs out with hiders uncaught.** Expected: round ends, survivors recorded with `caught=false`, CSV row still complete. Pinned in Task 16 `test_round_ends_on_timeout` and Task 18 `test_csv_row_with_survivors`.
5. **Same seed, two runs.** Expected: identical CSV rows. Pinned in Task 18 `test_same_seed_same_results` (scene test running two short rounds back to back in one world).

## File Map

| Path | Responsibility |
|---|---|
| `run_tests.sh` | Import + run gdUnit4 headless; exit code is the verdict |
| `scripts/robot/drive_model.gd` | `DriveModel` (RefCounted): unicycle math, wheel clamping, acceleration limiting |
| `scripts/robot/drive.gd` | `DriveBody` (RigidBody3D): applies `DriveModel` in `_integrate_forces`, exposes contacts + odometry stats |
| `scenes/robots/robot.tscn` | Robot scene: `DriveBody` root, chassis + wheel meshes, sensor nodes, `Brain` node |
| `scripts/robot/perception.gd` | `Blackboard` (RefCounted): sensor fields, memory, derived helpers, FSM outputs |
| `scripts/robot/lidar_blobs.gd` | `LidarBlobs` static clustering of lidar hits into robot-sized blobs |
| `scripts/robot/sensors/lidar.gd` / `vision_cone.gd` / `ultrasonic.gd` / `bumper.gd` | One Node3D each, `sense(bb)` writes to blackboard |
| `scripts/robot/sensors/debug_overlay.gd` | ImmediateMesh drawing of rays and cone |
| `scripts/robot/steering.gd` | `Steering` static reactive heading selection (pure) |
| `scripts/robot/motion.gd` | `Motion` (RefCounted): path following, avoidance blend, recovery, stuck detection |
| `scripts/robot/fsm/fsm.gd`, `fsm_state.gd` | `FSM`, `FSMState` base classes with transition log |
| `scripts/robot/fsm/seeker_states.gd` | Seeker states |
| `scripts/robot/fsm/hider_states.gd` | Hider states |
| `scripts/robot/fsm/hider_strategy.gd` | `HiderStrategy`, `StaticHider`, `DynamicHider` |
| `scripts/robot/robot.gd` | `RobotBrain` (Node): ticks sensors → FSM → motion → drive; role/strategy wiring |
| `scripts/game/nav_service.gd` | `NavService` (Node): bake navmesh from arena root, readiness, path queries |
| `scripts/game/visibility_grid.gd` | `VisibilityGrid` (RefCounted): walkable cells, openness, viewed-time, spot/patrol picking |
| `scripts/game/prop_layout.gd` | `PropLayout` pure seeded placement with overlap/corridor constraints |
| `scripts/game/prop_spawner.gd` | `PropSpawner` (Node3D): instantiates crates from a `PropLayout` |
| `scripts/game/round_config.gd` | `RoundConfig` (Resource) |
| `scripts/game/catch_rule.gd` | `CatchRule` pure predicate |
| `scripts/game/game_manager.gd` | `GameManager` (Node): round lifecycle, RNG, spawning, catch detection, results |
| `scripts/experiment/results_csv.gd` | `ResultsCsv` header/row formatting |
| `scripts/experiment/experiment_config.gd` | `ExperimentConfig` (Resource): named variants |
| `scripts/experiment/batch_runner.gd` | `BatchRunner` (Node): args, loop, CSV, time scale |
| `scripts/ui/hud.gd`, `camera_rig.gd` | HUD and camera |
| `scenes/arena/arena.tscn`, `scenes/props/crate.tscn`, `scenes/main.tscn`, `scenes/batch.tscn` | Scenes |
| `tests/unit/*_test.gd`, `tests/scene/*_test.gd`, `tests/fixtures/*.tscn` | Tests |
| `experiments/*.tres`, `experiments/analyze.py` | Experiment configs and analysis |

---

### Task 1: Project scaffolding and test runner

**Files:**
- Create: `run_tests.sh`, `tests/unit/smoke_test.gd`, `scripts/.gdkeep` (empty), `.gitignore` (modify)
- Create: `addons/gdUnit4/` (copied from gdUnit4 6.2.2 release)
- Modify: `project.godot`

**Interfaces:**
- Produces: `./run_tests.sh [extra gdUnit args]` exit 0 = all tests pass, 100 = failures.

- [ ] **Step 1: Install gdUnit4 6.2.2**

```bash
cd /mnt/LinuxGames/Robotics-Godot/engineering-project
TMP=$(mktemp -d)
curl -sL -o "$TMP/gdunit4.zip" https://github.com/MikeSchulze/gdUnit4/archive/refs/tags/v6.2.2.zip
unzip -q "$TMP/gdunit4.zip" -d "$TMP"
mkdir -p addons && cp -r "$TMP"/gdUnit4-*/addons/gdUnit4 addons/
grep version addons/gdUnit4/plugin.cfg   # expect version="6.2.2"
```

If the tag download fails, use the master zip already verified to work: `https://github.com/MikeSchulze/gdUnit4/archive/refs/heads/master.zip`.

- [ ] **Step 2: Enable plugin and layers in project.godot**

Append to `project.godot`:

```ini
[editor_plugins]

enabled=PackedStringArray("res://addons/gdUnit4/plugin.cfg")

[layer_names]

3d_physics/layer_1="walls"
3d_physics/layer_2="props"
3d_physics/layer_3="robots"

[navigation]

world/map_use_async_iterations=false
```

- [ ] **Step 3: Write run_tests.sh**

```bash
#!/usr/bin/env bash
# Runs the gdUnit4 suite headless. Exit 0 = pass, 100 = failures.
set -u
cd "$(dirname "$0")"
GODOT=./Godot_v4.7.2-stable_linux.x86_64
# Refresh the global script class cache so new class_name scripts resolve.
"$GODOT" --headless --path . --import >/dev/null 2>&1
"$GODOT" --headless --path . -s res://addons/gdUnit4/bin/GdUnitCmdTool.gd \
  --add res://tests --ignoreHeadlessMode "$@" 2>&1 | sed 's/\x1b\[[0-9;]*m//g' | grep -vE '^\s*$'
exit "${PIPESTATUS[0]}"
```

`chmod +x run_tests.sh`. Add to `.gitignore`: `reports/`.

- [ ] **Step 4: Write a smoke test that must fail first**

`tests/unit/smoke_test.gd`:

```gdscript
extends GdUnitTestSuite


func test_project_name() -> void:
	assert_str(ProjectSettings.get_setting("application/config/name")).is_equal("Robot Hide and Seek")
```

- [ ] **Step 5: Run, expect failure**

Run: `./run_tests.sh`
Expected: `1 failures`, exit code 100 (name is still "Engineering Project").

- [ ] **Step 6: Set the project name**

In `project.godot` change `config/name="Engineering Project"` to `config/name="Robot Hide and Seek"`.

- [ ] **Step 7: Run, expect pass**

Run: `./run_tests.sh` → `0 failures`, exit code 0.

- [ ] **Step 8: Commit**

```bash
git add -A && git commit -m "chore: add gdUnit4, test runner, collision layer names

Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>"
```

---

### Task 2: Differential drive math (`DriveModel`)

**Files:**
- Create: `scripts/robot/drive_model.gd`
- Test: `tests/unit/drive_model_test.gd`

**Interfaces:**
- Produces:
  - `class_name DriveModel extends RefCounted`
  - `var wheel_radius: float = 0.1`, `var track_width: float = 0.4`, `var max_wheel_speed: float = 25.0` (rad/s), `var max_wheel_accel: float = 60.0` (rad/s²)
  - `var wl: float`, `var wr: float` current wheel angular speeds
  - `static func wheels_from_body(v: float, w: float, r: float, track: float) -> Vector2` → (wl, wr)
  - `static func body_from_wheels(wl: float, wr: float, r: float, track: float) -> Vector2` → (v, w)
  - `func command(v: float, w: float, dt: float) -> void` updates `wl`, `wr` with clamp + rate limit
  - `func body_velocity() -> Vector2` → (v, w) from current wheels
  - `func max_linear_speed() -> float` = `wheel_radius * max_wheel_speed`

- [ ] **Step 1: Write failing tests**

`tests/unit/drive_model_test.gd`:

```gdscript
extends GdUnitTestSuite


func test_wheels_from_body_straight() -> void:
	var w: Vector2 = DriveModel.wheels_from_body(1.0, 0.0, 0.1, 0.4)
	assert_float(w.x).is_equal_approx(10.0, 1e-6)
	assert_float(w.y).is_equal_approx(10.0, 1e-6)


func test_wheels_from_body_turn_left_right_wheel_faster() -> void:
	var w: Vector2 = DriveModel.wheels_from_body(0.0, 1.0, 0.1, 0.4)
	# w = r(wr - wl)/L  =>  wr - wl = L/r = 4 ; symmetric => wl=-2, wr=2
	assert_float(w.x).is_equal_approx(-2.0, 1e-6)
	assert_float(w.y).is_equal_approx(2.0, 1e-6)


func test_roundtrip() -> void:
	var w: Vector2 = DriveModel.wheels_from_body(0.7, -0.3, 0.1, 0.4)
	var b: Vector2 = DriveModel.body_from_wheels(w.x, w.y, 0.1, 0.4)
	assert_float(b.x).is_equal_approx(0.7, 1e-6)
	assert_float(b.y).is_equal_approx(-0.3, 1e-6)


func test_command_clamps_wheel_speed() -> void:
	var m := DriveModel.new()
	m.max_wheel_speed = 10.0
	m.max_wheel_accel = 1e9
	m.command(100.0, 0.0, 0.1)
	assert_float(m.wl).is_equal_approx(10.0, 1e-6)
	assert_float(m.wr).is_equal_approx(10.0, 1e-6)


func test_command_rate_limits_acceleration() -> void:
	var m := DriveModel.new()
	m.max_wheel_accel = 10.0
	m.command(1.0, 0.0, 0.1)  # target wl = 10 rad/s, but only 1 rad/s change allowed in 0.1 s
	assert_float(m.wl).is_equal_approx(1.0, 1e-6)
	m.command(1.0, 0.0, 0.1)
	assert_float(m.wl).is_equal_approx(2.0, 1e-6)


func test_body_velocity_matches_command_when_unconstrained() -> void:
	var m := DriveModel.new()
	m.max_wheel_accel = 1e9
	m.command(0.5, 0.25, 0.016)
	var b: Vector2 = m.body_velocity()
	assert_float(b.x).is_equal_approx(0.5, 1e-6)
	assert_float(b.y).is_equal_approx(0.25, 1e-6)


func test_max_linear_speed() -> void:
	var m := DriveModel.new()
	m.wheel_radius = 0.1
	m.max_wheel_speed = 25.0
	assert_float(m.max_linear_speed()).is_equal_approx(2.5, 1e-6)
```

- [ ] **Step 2: Run, expect failure**

Run: `./run_tests.sh` → errors mentioning `DriveModel` not found (parse errors count as failures).

- [ ] **Step 3: Implement**

`scripts/robot/drive_model.gd`:

```gdscript
## Two-wheel differential drive (unicycle) model.
## v = r (wl + wr) / 2 ;  w = r (wr - wl) / L
class_name DriveModel
extends RefCounted

var wheel_radius: float = 0.1       # m
var track_width: float = 0.4        # m, distance between wheel contact points
var max_wheel_speed: float = 25.0   # rad/s
var max_wheel_accel: float = 60.0   # rad/s^2

var wl: float = 0.0  # left wheel angular speed, rad/s
var wr: float = 0.0  # right wheel angular speed, rad/s


static func wheels_from_body(v: float, w: float, r: float, track: float) -> Vector2:
	var wl_: float = (v - w * track * 0.5) / r
	var wr_: float = (v + w * track * 0.5) / r
	return Vector2(wl_, wr_)


static func body_from_wheels(wl_: float, wr_: float, r: float, track: float) -> Vector2:
	var v: float = r * (wl_ + wr_) * 0.5
	var w: float = r * (wr_ - wl_) / track
	return Vector2(v, w)


func command(v: float, w: float, dt: float) -> void:
	var target: Vector2 = wheels_from_body(v, w, wheel_radius, track_width)
	target.x = clampf(target.x, -max_wheel_speed, max_wheel_speed)
	target.y = clampf(target.y, -max_wheel_speed, max_wheel_speed)
	var max_delta: float = max_wheel_accel * dt
	wl = move_toward(wl, target.x, max_delta)
	wr = move_toward(wr, target.y, max_delta)


func body_velocity() -> Vector2:
	return body_from_wheels(wl, wr, wheel_radius, track_width)


func max_linear_speed() -> float:
	return wheel_radius * max_wheel_speed
```

- [ ] **Step 4: Run, expect pass**

Run: `./run_tests.sh` → 8 test cases, 0 failures.

- [ ] **Step 5: Commit**

```bash
git add scripts/robot/drive_model.gd tests/unit/drive_model_test.gd
git commit -m "feat: differential drive model with clamping and rate limiting

Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>"
```

---

### Task 3: Drive body and robot scene

**Files:**
- Create: `scripts/robot/drive.gd`, `scenes/robots/robot.tscn`, `tests/fixtures/flat_floor.tscn`
- Test: `tests/scene/drive_body_test.gd`

**Interfaces:**
- Consumes: `DriveModel`.
- Produces:
  - `class_name DriveBody extends RigidBody3D` (root of `robot.tscn`)
  - `@export var role: String = "hider"`, `var model: DriveModel`
  - `func set_command(v: float, w: float) -> void` (desired body velocity, applied on next physics tick)
  - `func yaw() -> float`, `func forward() -> Vector3` (unit, world)
  - `var distance_travelled: float`, `var collision_count: int`
  - `var contacts: Array[Dictionary]` each `{"normal": Vector3 (world, pointing away from other body into us), "position": Vector3 (world), "collider": Object}` refreshed each physics tick
  - Robot scene node paths: `Chassis` (MeshInstance3D), `WheelL`, `WheelR` (MeshInstance3D), `Sensors` (Node3D at eye height 0.25), `Brain` (Node, script attached in Task 14)
  - Robot is in group `robots`, collision layer 3, mask 1|2|3.

- [ ] **Step 1: Write the fixture floor scene**

`tests/fixtures/flat_floor.tscn`:

```
[gd_scene load_steps=3 format=3]

[sub_resource type="BoxShape3D" id="floor_shape"]
size = Vector3(60, 1, 60)

[sub_resource type="BoxShape3D" id="wall_shape"]
size = Vector3(0.2, 2, 20)

[node name="FlatFloor" type="Node3D"]

[node name="Floor" type="StaticBody3D" parent="."]
transform = Transform3D(1, 0, 0, 0, 1, 0, 0, 0, 1, 0, -0.5, 0)
collision_layer = 1
collision_mask = 0

[node name="FloorShape" type="CollisionShape3D" parent="Floor"]
shape = SubResource("floor_shape")

[node name="WallEast" type="StaticBody3D" parent="."]
transform = Transform3D(1, 0, 0, 0, 1, 0, 0, 0, 1, 5, 1, 0)
collision_layer = 1
collision_mask = 0

[node name="WallShape" type="CollisionShape3D" parent="WallEast"]
shape = SubResource("wall_shape")
```

(The wall at x = +5 spanning z ∈ [-10, 10] is reused by the lidar test in Task 6.)

- [ ] **Step 2: Write the failing scene test**

`tests/scene/drive_body_test.gd`:

```gdscript
extends GdUnitTestSuite

const ROBOT := preload("res://scenes/robots/robot.tscn")
const FLOOR := preload("res://tests/fixtures/flat_floor.tscn")


func _spawn(runner: GdUnitSceneRunner, pos: Vector3) -> DriveBody:
	var robot: DriveBody = ROBOT.instantiate()
	robot.position = pos
	runner.scene().add_child(robot)
	return robot


func test_forward_command_moves_along_minus_z() -> void:
	var runner := scene_runner(FLOOR.instantiate())
	var robot := _spawn(runner, Vector3(0, 0.3, 0))
	await runner.simulate_frames(10)  # settle on floor
	var start: Vector3 = robot.global_position
	robot.set_command(1.0, 0.0)
	await runner.simulate_frames(60)  # 1 s at 60 Hz
	var moved: Vector3 = robot.global_position - start
	assert_float(moved.z).is_between(-1.1, -0.8)
	assert_float(absf(moved.x)).is_less(0.05)
	assert_float(robot.distance_travelled).is_between(0.8, 1.1)


func test_turn_command_rotates_in_place() -> void:
	var runner := scene_runner(FLOOR.instantiate())
	var robot := _spawn(runner, Vector3(0, 0.3, 0))
	await runner.simulate_frames(10)
	var start: Vector3 = robot.global_position
	robot.set_command(0.0, 1.0)
	await runner.simulate_frames(60)
	assert_float(robot.yaw()).is_between(0.9, 1.1)
	assert_float(robot.global_position.distance_to(start)).is_less(0.05)


func test_stop_command_holds_still() -> void:
	var runner := scene_runner(FLOOR.instantiate())
	var robot := _spawn(runner, Vector3(0, 0.3, 0))
	await runner.simulate_frames(10)
	robot.set_command(1.0, 0.0)
	await runner.simulate_frames(30)
	robot.set_command(0.0, 0.0)
	await runner.simulate_frames(30)
	var p: Vector3 = robot.global_position
	await runner.simulate_frames(30)
	assert_float(robot.global_position.distance_to(p)).is_less(0.02)


func test_contacts_and_collision_count_when_driving_into_wall() -> void:
	var runner := scene_runner(FLOOR.instantiate())
	var robot := _spawn(runner, Vector3(3.5, 0.3, 0))
	await runner.simulate_frames(10)
	robot.rotation.y = -PI / 2  # face +X toward the wall at x=5
	robot.set_command(1.5, 0.0)
	await runner.simulate_frames(120)
	assert_int(robot.collision_count).is_greater_equal(1)
	assert_array(robot.contacts).is_not_empty()
	var n: Vector3 = robot.contacts[0]["normal"]
	assert_float(n.x).is_less(-0.9)  # wall pushes us back toward -X


func test_robot_is_in_robots_group_and_layer_3() -> void:
	var runner := scene_runner(FLOOR.instantiate())
	var robot := _spawn(runner, Vector3(0, 0.3, 0))
	await runner.simulate_frames(1)
	assert_bool(robot.is_in_group("robots")).is_true()
	assert_int(robot.collision_layer).is_equal(4)
```

- [ ] **Step 3: Run, expect failure**

Run: `./run_tests.sh` → failures/errors (scene missing, `DriveBody` unknown).

- [ ] **Step 4: Implement drive.gd**

`scripts/robot/drive.gd`:

```gdscript
## Differential-drive chassis. Applies DriveModel wheel speeds as body velocity
## inside _integrate_forces so collisions with props and robots stay physical.
## Requires a frictionless physics material (set in robot.tscn); see NOTES.
class_name DriveBody
extends RigidBody3D

@export var role: String = "hider"  # "seeker" | "hider"
@export var wheel_radius: float = 0.1
@export var track_width: float = 0.4
@export var max_wheel_speed: float = 25.0
@export var max_wheel_accel: float = 60.0

var model: DriveModel = DriveModel.new()
var distance_travelled: float = 0.0
var collision_count: int = 0
var contacts: Array[Dictionary] = []

var _cmd_v: float = 0.0
var _cmd_w: float = 0.0
var _had_contact: bool = false
var _last_pos: Vector3


func _ready() -> void:
	model.wheel_radius = wheel_radius
	model.track_width = track_width
	model.max_wheel_speed = max_wheel_speed
	model.max_wheel_accel = max_wheel_accel
	add_to_group("robots")
	contact_monitor = true
	max_contacts_reported = 8
	axis_lock_angular_x = true
	axis_lock_angular_z = true
	_last_pos = global_position


func set_command(v: float, w: float) -> void:
	_cmd_v = v
	_cmd_w = w


func yaw() -> float:
	return global_rotation.y


func forward() -> Vector3:
	return -global_transform.basis.z


func _integrate_forces(state: PhysicsDirectBodyState3D) -> void:
	var dt: float = state.step
	model.command(_cmd_v, _cmd_w, dt)
	var vw: Vector2 = model.body_velocity()
	var lv: Vector3 = forward() * vw.x
	lv.y = state.linear_velocity.y  # keep gravity
	state.linear_velocity = lv
	state.angular_velocity = Vector3(0.0, vw.y, 0.0)

	# Contacts (world space), filtered to roughly horizontal normals = not the floor.
	contacts.clear()
	for i in state.get_contact_count():
		var n: Vector3 = state.get_contact_local_normal(i)
		if absf(n.y) > 0.5:
			continue
		contacts.append({
			"normal": n,
			"position": state.get_contact_local_position(i),
			"collider": state.get_contact_collider_object(i),
		})
	var has_contact: bool = not contacts.is_empty()
	if has_contact and not _had_contact:
		collision_count += 1
	_had_contact = has_contact

	# Odometry (horizontal distance only).
	var p: Vector3 = state.transform.origin
	var d: Vector3 = p - _last_pos
	d.y = 0.0
	distance_travelled += d.length()
	_last_pos = p
```

Note: `get_contact_local_normal` / `get_contact_local_position` are world-space despite the name (Godot docs). If the wall test's normal sign is inverted, negate `n` once here and keep the test as written.

- [ ] **Step 5: Write robot.tscn**

`scenes/robots/robot.tscn`:

```
[gd_scene load_steps=8 format=3]

[ext_resource type="Script" path="res://scripts/robot/drive.gd" id="drive"]

[sub_resource type="PhysicsMaterial" id="frictionless"]
friction = 0.0
bounce = 0.0

[sub_resource type="BoxShape3D" id="chassis_shape"]
size = Vector3(0.4, 0.2, 0.5)

[sub_resource type="BoxMesh" id="chassis_mesh"]
size = Vector3(0.4, 0.2, 0.5)

[sub_resource type="StandardMaterial3D" id="chassis_mat"]
albedo_color = Color(0.2, 0.6, 0.9, 1)

[sub_resource type="CylinderMesh" id="wheel_mesh"]
top_radius = 0.1
bottom_radius = 0.1
height = 0.05

[sub_resource type="StandardMaterial3D" id="wheel_mat"]
albedo_color = Color(0.1, 0.1, 0.1, 1)

[node name="Robot" type="RigidBody3D"]
collision_layer = 4
collision_mask = 7
mass = 5.0
physics_material_override = SubResource("frictionless")
can_sleep = false
script = ExtResource("drive")

[node name="CollisionShape3D" type="CollisionShape3D" parent="."]
shape = SubResource("chassis_shape")

[node name="Chassis" type="MeshInstance3D" parent="."]
mesh = SubResource("chassis_mesh")
surface_material_override/0 = SubResource("chassis_mat")

[node name="WheelL" type="MeshInstance3D" parent="."]
transform = Transform3D(0, -1, 0, 1, 0, 0, 0, 0, 1, -0.225, -0.05, 0)
mesh = SubResource("wheel_mesh")
surface_material_override/0 = SubResource("wheel_mat")

[node name="WheelR" type="MeshInstance3D" parent="."]
transform = Transform3D(0, -1, 0, 1, 0, 0, 0, 0, 1, 0.225, -0.05, 0)
mesh = SubResource("wheel_mesh")
surface_material_override/0 = SubResource("wheel_mat")

[node name="Nose" type="MeshInstance3D" parent="."]
transform = Transform3D(0.5, 0, 0, 0, 0.5, 0, 0, 0, 0.5, 0, 0.12, -0.2)
mesh = SubResource("chassis_mesh")
surface_material_override/0 = SubResource("wheel_mat")

[node name="Sensors" type="Node3D" parent="."]
transform = Transform3D(1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0.15, 0)

[node name="Brain" type="Node" parent="."]
```

The `Nose` block marks the front (-Z) so direction is readable in screenshots.

- [ ] **Step 6: Run, expect pass**

Run: `./run_tests.sh` → all pass. If `test_turn_command_rotates_in_place` yaw is off by more than 10 %, check that `physics_material_override` is applied (see NOTES item 1) before touching the model.

- [ ] **Step 7: Visual check via computer-use-vnc**

Open `scenes/robots/robot.tscn` in the running editor (double-click in FileSystem dock), confirm the chassis, two wheels and nose render. Take a screenshot into `docs/screenshots/robot.png` (create the dir). This is a check, not a test; no code change expected.

- [ ] **Step 8: Commit**

```bash
git add scripts/robot/drive.gd scenes/robots/robot.tscn tests/ docs/screenshots
git commit -m "feat: differential drive body and robot scene

Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>"
```

---

### Task 4: Arena scene and navigation service

**Files:**
- Create: `scenes/arena/arena.tscn`, `scripts/game/nav_service.gd`, `scripts/game/arena.gd`
- Test: `tests/scene/nav_service_test.gd`

**Interfaces:**
- Produces:
  - `class_name Arena extends Node3D` (root of `arena.tscn`) with children `Static` (Node3D holding all wall StaticBody3Ds), `Props` (Node3D, empty, filled by spawner), `Robots` (Node3D), `SeekerSpawn` (Marker3D), `HiderSpawns` (Node3D with 5 Marker3D children), `NavService` (Node, script `nav_service.gd`).
  - `func bounds() -> Rect2` → arena floor rectangle in XZ: `Rect2(-15, -15, 30, 30)`
  - `func wall_rects() -> Array[Rect2]` → XZ footprints of every interior and outer wall (used by prop layout and visibility grid), derived from the `Static` children's BoxShape3D sizes and transforms.
  - `func hider_spawn_points() -> Array[Vector3]`, `func seeker_spawn_point() -> Vector3`
  - `class_name NavService extends Node`
    - `func bake(source_root: Node3D) -> void` bakes walls-only navmesh (layer 1), creates/replaces its `NavigationRegion3D` child
    - `func is_ready() -> bool` true once a trivial path query between two known points returns ≥ 2 points
    - `func get_path(from: Vector3, to: Vector3) -> PackedVector3Array`
    - `func closest_point(p: Vector3) -> Vector3`
    - `@export var agent_radius: float = 0.35`, `@export var cell_size: float = 0.25`

- [ ] **Step 1: Write the failing scene test**

`tests/scene/nav_service_test.gd`:

```gdscript
extends GdUnitTestSuite

const ARENA := preload("res://scenes/arena/arena.tscn")


func _ready_arena() -> Array:  # [runner, arena]
	var runner := scene_runner(ARENA.instantiate())
	var arena: Arena = runner.scene()
	arena.nav.bake(arena.static_root)
	for i in 30:
		await runner.simulate_frames(1)
		if arena.nav.is_ready():
			break
	return [runner, arena]


func test_nav_becomes_ready_within_30_frames() -> void:
	var r := await _ready_arena()
	var arena: Arena = r[1]
	assert_bool(arena.nav.is_ready()).is_true()


func test_path_between_seeker_and_every_hider_spawn() -> void:
	var r := await _ready_arena()
	var arena: Arena = r[1]
	for h in arena.hider_spawn_points():
		var path: PackedVector3Array = arena.nav.get_path(arena.seeker_spawn_point(), h)
		assert_int(path.size()).append_failure_message("no path to %s" % h).is_greater_equal(2)
		assert_float(path[-1].distance_to(h)).is_less(1.0)


func test_wall_rects_and_bounds() -> void:
	var runner := scene_runner(ARENA.instantiate())
	var arena: Arena = runner.scene()
	assert_that(arena.bounds()).is_equal(Rect2(-15, -15, 30, 30))
	var rects: Array[Rect2] = arena.wall_rects()
	assert_int(rects.size()).is_greater_equal(8)  # 4 outer + interior walls
	for rct in rects:
		assert_bool(Rect2(-15.5, -15.5, 31, 31).encloses(rct)).is_true()


func test_hider_spawns_not_visible_from_seeker_spawn() -> void:
	var r := await _ready_arena()
	var runner: GdUnitSceneRunner = r[0]
	var arena: Arena = r[1]
	var space := arena.get_world_3d().direct_space_state
	var eye: Vector3 = arena.seeker_spawn_point() + Vector3(0, 0.3, 0)
	for h in arena.hider_spawn_points():
		var q := PhysicsRayQueryParameters3D.create(eye, h + Vector3(0, 0.3, 0), 1)
		var hit: Dictionary = space.intersect_ray(q)
		assert_bool(hit.is_empty()).append_failure_message("spawn %s is visible from seeker spawn" % h).is_false()
```

- [ ] **Step 2: Run, expect failure**

Run: `./run_tests.sh` → `Arena`/scene missing.

- [ ] **Step 3: Implement nav_service.gd**

`scripts/game/nav_service.gd`:

```gdscript
## Bakes a walls-only navmesh from a given root and answers path queries.
## Readiness must be polled: the NavigationServer needs a few physics frames
## after the region is added before queries return anything (see NOTES).
class_name NavService
extends Node

@export var agent_radius: float = 0.35
@export var cell_size: float = 0.25

var _region: NavigationRegion3D
var _probe_a: Vector3
var _probe_b: Vector3
var _ready_flag: bool = false


func bake(source_root: Node3D) -> void:
	var nm := NavigationMesh.new()
	nm.agent_radius = agent_radius
	nm.cell_size = cell_size
	nm.cell_height = 0.25
	nm.agent_height = 0.5
	nm.geometry_parsed_geometry_type = NavigationMesh.PARSED_GEOMETRY_STATIC_COLLIDERS
	nm.geometry_collision_mask = 1  # walls + floor only
	var src := NavigationMeshSourceGeometryData3D.new()
	NavigationServer3D.parse_source_geometry_data(nm, src, source_root)
	NavigationServer3D.bake_from_source_geometry_data(nm, src)
	if _region:
		_region.queue_free()
	_region = NavigationRegion3D.new()
	_region.navigation_mesh = nm
	add_child(_region)
	var map: RID = get_viewport().world_3d.navigation_map
	NavigationServer3D.map_set_use_async_iterations(map, false)
	_ready_flag = false


func set_probe_points(a: Vector3, b: Vector3) -> void:
	_probe_a = a
	_probe_b = b


func is_ready() -> bool:
	if _ready_flag:
		return true
	if _region == null:
		return false
	var path: PackedVector3Array = get_path(_probe_a, _probe_b)
	_ready_flag = path.size() >= 2
	return _ready_flag


func get_path(from: Vector3, to: Vector3) -> PackedVector3Array:
	var map: RID = get_viewport().world_3d.navigation_map
	return NavigationServer3D.map_get_path(map, from, to, true)


func closest_point(p: Vector3) -> Vector3:
	var map: RID = get_viewport().world_3d.navigation_map
	return NavigationServer3D.map_get_closest_point(map, p)
```

- [ ] **Step 4: Implement arena.gd**

`scripts/game/arena.gd`:

```gdscript
## Hand-built arena. Walls live under Static; Props and Robots are filled at runtime.
class_name Arena
extends Node3D

@onready var static_root: Node3D = $Static
@onready var props_root: Node3D = $Props
@onready var robots_root: Node3D = $Robots
@onready var nav: NavService = $NavService


func _ready() -> void:
	var hs: Array[Vector3] = hider_spawn_points()
	nav.set_probe_points(seeker_spawn_point(), hs[0])


func bounds() -> Rect2:
	return Rect2(-15.0, -15.0, 30.0, 30.0)


func seeker_spawn_point() -> Vector3:
	return ($SeekerSpawn as Marker3D).global_position


func hider_spawn_points() -> Array[Vector3]:
	var out: Array[Vector3] = []
	for m in $HiderSpawns.get_children():
		out.append((m as Marker3D).global_position)
	return out


## XZ footprints of all wall boxes (axis-aligned; walls are never rotated).
func wall_rects() -> Array[Rect2]:
	var out: Array[Rect2] = []
	for body in static_root.get_children():
		if body.name == "Floor":
			continue
		for shape_node in body.get_children():
			var cs := shape_node as CollisionShape3D
			if cs == null or not (cs.shape is BoxShape3D):
				continue
			var size: Vector3 = (cs.shape as BoxShape3D).size
			var c: Vector3 = cs.global_position
			out.append(Rect2(c.x - size.x * 0.5, c.z - size.z * 0.5, size.x, size.z))
	return out
```

- [ ] **Step 5: Write arena.tscn**

Layout (top view, X right, Z down, 30 × 30 m, walls 2 m high, 0.3 m thick). Interior walls form four rooms around a central pillar block with openings at least 2.5 m wide:

```
   x: -15    -8 -5 -1 0 1  5  8 9     15
z=-15  ┌─────────────┬──────────────────┐
       │  room NW    │   room NE  │     │   NookNE: x=9, z∈[-15,-10]
       │    H1       │      H2    │     │
z=-8   │             ┴                  │   MidV_N: x=0, z∈[-15,-8]
z=-5   │───────  ────  ────  ───────────│   openings at x∈[-8,-5], [-1,1], [5,8]
       │                                │
z= 0   │     S       █            H5    │   pillar 2x2 at origin, seeker spawn S=(-7,0)
       │                                │
z= 5   │───────  ────  ────  ───────────│   mirrored south divider
z= 8   │             ┬                  │   MidV_S: x=0, z∈[8,15]
       │  room SW    │   room SE        │
       │     │ H4    │      H3          │   NookSW: x=-9, z∈[10,15]
z=15   └─────┴───────┴──────────────────┘
```

Each wall is a `StaticBody3D` (layer 1, mask 0) with one `CollisionShape3D` + `MeshInstance3D` sharing a `BoxShape3D`/`BoxMesh` of the same size. Write the file with this header and one block per wall; sizes and positions below are the source of truth:

| Name | Size (x, y, z) | Position (x, y, z) |
|---|---|---|
| Floor | (30, 1, 30) | (0, -0.5, 0) |
| OuterN | (30.6, 2, 0.3) | (0, 1, -15.15) |
| OuterS | (30.6, 2, 0.3) | (0, 1, 15.15) |
| OuterW | (0.3, 2, 30.6) | (-15.15, 1, 0) |
| OuterE | (0.3, 2, 30.6) | (15.15, 1, 0) |
| MidV_N | (0.3, 2, 7) | (0, 1, -11.5) |
| MidV_S | (0.3, 2, 7) | (0, 1, 11.5) |
| Pillar | (2, 2, 2) | (0, 1, 0) |
| WestH_N_a | (7, 2, 0.3) | (-11.5, 1, -5) |
| WestH_N_b | (4, 2, 0.3) | (-3, 1, -5) |
| EastH_N_a | (4, 2, 0.3) | (3, 1, -5) |
| EastH_N_b | (7, 2, 0.3) | (11.5, 1, -5) |
| WestH_S_a | (7, 2, 0.3) | (-11.5, 1, 5) |
| WestH_S_b | (4, 2, 0.3) | (-3, 1, 5) |
| EastH_S_a | (4, 2, 0.3) | (3, 1, 5) |
| EastH_S_b | (7, 2, 0.3) | (11.5, 1, 5) |
| NookNE | (0.3, 2, 5) | (9, 1, -12.5) |
| NookSW | (0.3, 2, 5) | (-9, 1, 12.5) |

Markers: `SeekerSpawn` at (-7, 0, 0). `HiderSpawns/H1` (-12, 0, -12), `H2` (13, 0, -11), `H3` (13, 0, 11), `H4` (-12, 0, 12), `H5` (12, 0, -2). These were chosen so the straight line from the seeker spawn to each marker crosses a wall segment: H1/H4 cross the z = ±5 dividers at x ≈ -9.1 (inside the 7 m west segments), H2/H3 cross them at x ≈ 2.1 (inside the 4 m east segments), H5 is behind the pillar. Openings in each divider: x ∈ [-8, -5], [-1, 1], [5, 8]. Root `Arena` has `script = ExtResource(arena.gd)`; `NavService` node has `script = ExtResource(nav_service.gd)`. Add a `DirectionalLight3D` (rotation x = -60°) and a `WorldEnvironment` with a plain `Environment` (ambient light energy 1.0, background color sky-blue) so the scene is visible in the editor and screenshots.

Example of one wall block (repeat per row of the table, substituting name/size/position; each wall gets its own `BoxShape3D`/`BoxMesh` sub_resource pair):

```
[node name="OuterN" type="StaticBody3D" parent="Static"]
transform = Transform3D(1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, -15.15)
collision_layer = 1
collision_mask = 0

[node name="Shape" type="CollisionShape3D" parent="Static/OuterN"]
shape = SubResource("shape_OuterN")

[node name="Mesh" type="MeshInstance3D" parent="Static/OuterN"]
mesh = SubResource("mesh_OuterN")
surface_material_override/0 = SubResource("wall_mat")
```

Generate the file with a short throwaway Python script in the scratchpad from the table rather than typing 18 blocks by hand; commit only the resulting `.tscn`.

- [ ] **Step 6: Run, expect pass**

Run: `./run_tests.sh`. If `test_hider_spawns_not_visible_from_seeker_spawn` fails for a marker, move that marker deeper into its room (keep it ≥ 1 m from walls) rather than moving walls.

- [ ] **Step 7: Visual check via computer-use-vnc**

Open `arena.tscn` in the editor, orbit the 3D view to a top-down angle, confirm the four rooms, openings, pillar and nooks match the ASCII layout. Enable *Debug → Visible Navigation* then run the scene (F6) after temporarily adding a `_ready` call `nav.bake(static_root)` in a scratch copy, or simply check in Task 15's full round. Screenshot to `docs/screenshots/arena_top.png`.

- [ ] **Step 8: Commit**

```bash
git add scenes/arena scripts/game/arena.gd scripts/game/nav_service.gd tests/scene/nav_service_test.gd docs/screenshots
git commit -m "feat: hand-built arena scene and navigation service

Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>"
```

---

### Task 5: Props: layout generator and spawner

**Files:**
- Create: `scripts/game/prop_layout.gd`, `scripts/game/prop_spawner.gd`, `scenes/props/crate.tscn`
- Test: `tests/unit/prop_layout_test.gd`, `tests/scene/prop_spawner_test.gd`

**Interfaces:**
- Consumes: `Arena.bounds()`, `Arena.wall_rects()`, spawn points.
- Produces:
  - `class_name PropLayout extends RefCounted`
    - `static func generate(rng: RandomNumberGenerator, count: int, bounds: Rect2, walls: Array[Rect2], keep_clear: Array[Vector3], sizes: Array[Vector2], min_gap: float = 1.5, max_attempts: int = 2000) -> Array[Dictionary]` each `{"pos": Vector2 (XZ), "size": Vector2 (XZ footprint), "rot": float (0 or PI/2)}`
    - `static func overlaps(a: Rect2, b: Rect2, gap: float) -> bool`
  - `class_name PropSpawner extends Node3D` with `func spawn(layout: Array[Dictionary], parent: Node3D) -> void` and `func clear(parent: Node3D) -> void`
  - `crate.tscn`: `StaticBody3D` root, layer 2, mask 0, `BoxShape3D` + `BoxMesh` 1 × 1 × 1 m; spawner scales footprint via `scale` to the layout size and height 1.0. Crates are in group `props`.
  - Crate footprint sizes used by the game: `[Vector2(1.0, 1.0), Vector2(1.5, 1.0), Vector2(2.0, 1.0), Vector2(1.2, 1.2)]`. Minimum dimension 1.0 m so lidar blob detection (Task 7) can tell robots (0.4–0.5 m) from crates.

- [ ] **Step 1: Write failing unit tests**

`tests/unit/prop_layout_test.gd`:

```gdscript
extends GdUnitTestSuite

const BOUNDS := Rect2(-15, -15, 30, 30)
const SIZES: Array[Vector2] = [Vector2(1, 1), Vector2(1.5, 1), Vector2(2, 1), Vector2(1.2, 1.2)]


func _walls() -> Array[Rect2]:
	return [Rect2(-15.3, -15.3, 30.6, 0.3), Rect2(-1, -1, 2, 2), Rect2(-15, -5.15, 6, 0.3)]


func _rng(seed: int) -> RandomNumberGenerator:
	var r := RandomNumberGenerator.new()
	r.seed = seed
	return r


func _rect(p: Dictionary) -> Rect2:
	var s: Vector2 = p["size"] if p["rot"] == 0.0 else Vector2(p["size"].y, p["size"].x)
	return Rect2(p["pos"] - s * 0.5, s)


func test_generates_requested_count() -> void:
	var layout := PropLayout.generate(_rng(1), 12, BOUNDS, _walls(), [], SIZES)
	assert_int(layout.size()).is_equal(12)


func test_props_do_not_overlap_each_other_or_walls() -> void:
	var layout := PropLayout.generate(_rng(2), 12, BOUNDS, _walls(), [], SIZES, 1.5)
	for i in layout.size():
		for j in range(i + 1, layout.size()):
			assert_bool(PropLayout.overlaps(_rect(layout[i]), _rect(layout[j]), 1.5)).is_false()
		for w in _walls():
			assert_bool(PropLayout.overlaps(_rect(layout[i]), w, 1.5)).is_false()


func test_props_stay_inside_bounds() -> void:
	var layout := PropLayout.generate(_rng(3), 12, BOUNDS, _walls(), [], SIZES)
	for p in layout:
		assert_bool(BOUNDS.grow(-1.0).encloses(_rect(p))).is_true()


func test_keep_clear_points_respected() -> void:
	var clear: Array[Vector3] = [Vector3(-7, 0, 0), Vector3(12, 0, 12)]
	var layout := PropLayout.generate(_rng(4), 12, BOUNDS, _walls(), clear, SIZES)
	for p in layout:
		for c in clear:
			var cr := Rect2(Vector2(c.x, c.z) - Vector2(1, 1), Vector2(2, 2))
			assert_bool(PropLayout.overlaps(_rect(p), cr, 1.5)).is_false()


func test_same_seed_same_layout() -> void:
	var a := PropLayout.generate(_rng(7), 12, BOUNDS, _walls(), [], SIZES)
	var b := PropLayout.generate(_rng(7), 12, BOUNDS, _walls(), [], SIZES)
	assert_array(a).is_equal(b)


func test_different_seed_different_layout() -> void:
	var a := PropLayout.generate(_rng(7), 12, BOUNDS, _walls(), [], SIZES)
	var b := PropLayout.generate(_rng(8), 12, BOUNDS, _walls(), [], SIZES)
	assert_array(a).is_not_equal(b)


func test_spawner_terminates_on_crowded_arena() -> void:
	# Tiny arena, huge gap: cannot fit 12. Must return fewer, not hang.
	var layout := PropLayout.generate(_rng(5), 12, Rect2(-3, -3, 6, 6), [], [], SIZES, 3.0, 500)
	assert_int(layout.size()).is_less(12)


func test_overlaps_uses_gap() -> void:
	var a := Rect2(0, 0, 1, 1)
	var b := Rect2(2, 0, 1, 1)  # 1 m apart
	assert_bool(PropLayout.overlaps(a, b, 0.5)).is_false()
	assert_bool(PropLayout.overlaps(a, b, 1.5)).is_true()
```

- [ ] **Step 2: Run, expect failure**

Run: `./run_tests.sh` → `PropLayout` unknown.

- [ ] **Step 3: Implement prop_layout.gd**

`scripts/game/prop_layout.gd`:

```gdscript
## Seeded random placement of rectangular props on the XZ plane.
## Rejects candidates overlapping walls, other props, or keep-clear points,
## with a bounded attempt budget so crowded arenas terminate.
class_name PropLayout
extends RefCounted


static func overlaps(a: Rect2, b: Rect2, gap: float) -> bool:
	return a.grow(gap * 0.5).intersects(b.grow(gap * 0.5))


static func _footprint(pos: Vector2, size: Vector2, rot: float) -> Rect2:
	var s: Vector2 = size if rot == 0.0 else Vector2(size.y, size.x)
	return Rect2(pos - s * 0.5, s)


static func generate(rng: RandomNumberGenerator, count: int, bounds: Rect2,
		walls: Array[Rect2], keep_clear: Array[Vector3], sizes: Array[Vector2],
		min_gap: float = 1.5, max_attempts: int = 2000) -> Array[Dictionary]:
	var out: Array[Dictionary] = []
	var placed: Array[Rect2] = []
	var clear_rects: Array[Rect2] = []
	for c in keep_clear:
		clear_rects.append(Rect2(Vector2(c.x, c.z) - Vector2(1, 1), Vector2(2, 2)))
	var inner: Rect2 = bounds.grow(-1.0)
	var attempts: int = 0
	while out.size() < count and attempts < max_attempts:
		attempts += 1
		var size: Vector2 = sizes[rng.randi_range(0, sizes.size() - 1)]
		var rot: float = 0.0 if rng.randf() < 0.5 else PI * 0.5
		var pos := Vector2(
			rng.randf_range(inner.position.x + 1.0, inner.end.x - 1.0),
			rng.randf_range(inner.position.y + 1.0, inner.end.y - 1.0))
		var r: Rect2 = _footprint(pos, size, rot)
		if not inner.encloses(r):
			continue
		var ok: bool = true
		for w in walls:
			if overlaps(r, w, min_gap):
				ok = false
				break
		if ok:
			for p in placed:
				if overlaps(r, p, min_gap):
					ok = false
					break
		if ok:
			for c in clear_rects:
				if overlaps(r, c, min_gap):
					ok = false
					break
		if not ok:
			continue
		placed.append(r)
		out.append({"pos": pos, "size": size, "rot": rot})
	return out
```

- [ ] **Step 4: Run unit tests, expect pass**

Run: `./run_tests.sh` → prop_layout tests pass.

- [ ] **Step 5: Write crate.tscn and prop_spawner.gd**

`scenes/props/crate.tscn`:

```
[gd_scene load_steps=4 format=3]

[sub_resource type="BoxShape3D" id="shape"]
size = Vector3(1, 1, 1)

[sub_resource type="BoxMesh" id="mesh"]
size = Vector3(1, 1, 1)

[sub_resource type="StandardMaterial3D" id="mat"]
albedo_color = Color(0.65, 0.45, 0.2, 1)

[node name="Crate" type="StaticBody3D" groups=["props"]]
collision_layer = 2
collision_mask = 0

[node name="Shape" type="CollisionShape3D" parent="."]
shape = SubResource("shape")

[node name="Mesh" type="MeshInstance3D" parent="."]
mesh = SubResource("mesh")
surface_material_override/0 = SubResource("mat")
```

`scripts/game/prop_spawner.gd`:

```gdscript
## Instantiates crate scenes from a PropLayout result.
class_name PropSpawner
extends Node3D

const CRATE := preload("res://scenes/props/crate.tscn")
const SIZES: Array[Vector2] = [Vector2(1.0, 1.0), Vector2(1.5, 1.0), Vector2(2.0, 1.0), Vector2(1.2, 1.2)]
const HEIGHT: float = 1.0


func clear(parent: Node3D) -> void:
	for c in parent.get_children():
		parent.remove_child(c)
		c.queue_free()


func spawn(layout: Array[Dictionary], parent: Node3D) -> void:
	for i in layout.size():
		var p: Dictionary = layout[i]
		var crate: StaticBody3D = CRATE.instantiate()
		crate.name = "Crate%d" % i
		parent.add_child(crate)
		crate.scale = Vector3(p["size"].x, HEIGHT, p["size"].y)
		crate.rotation.y = p["rot"]
		crate.position = Vector3(p["pos"].x, HEIGHT * 0.5, p["pos"].y)
```

Scaling a `StaticBody3D` scales its shape in Jolt; this is acceptable for static boxes.

- [ ] **Step 6: Write the spawner scene test**

`tests/scene/prop_spawner_test.gd`:

```gdscript
extends GdUnitTestSuite

const ARENA := preload("res://scenes/arena/arena.tscn")


func test_spawn_places_crates_that_block_rays() -> void:
	var runner := scene_runner(ARENA.instantiate())
	var arena: Arena = runner.scene()
	var spawner := PropSpawner.new()
	arena.add_child(spawner)
	var rng := RandomNumberGenerator.new()
	rng.seed = 11
	var layout := PropLayout.generate(rng, 12, arena.bounds(), arena.wall_rects(),
		[arena.seeker_spawn_point()] + arena.hider_spawn_points(), PropSpawner.SIZES)
	spawner.spawn(layout, arena.props_root)
	await runner.simulate_frames(2)
	assert_int(arena.props_root.get_child_count()).is_equal(layout.size())
	var space := arena.get_world_3d().direct_space_state
	var p: Dictionary = layout[0]
	var from := Vector3(p["pos"].x, 0.5, p["pos"].y) + Vector3(5, 0, 0)
	var q := PhysicsRayQueryParameters3D.create(from, Vector3(p["pos"].x, 0.5, p["pos"].y), 2)
	var hit: Dictionary = space.intersect_ray(q)
	assert_bool(hit.is_empty()).is_false()
	assert_bool((hit["collider"] as Node).is_in_group("props")).is_true()
	spawner.clear(arena.props_root)
	await runner.simulate_frames(1)
	assert_int(arena.props_root.get_child_count()).is_equal(0)
```

- [ ] **Step 7: Run, expect pass**

Run: `./run_tests.sh` → all pass.

- [ ] **Step 8: Commit**

```bash
git add scripts/game/prop_layout.gd scripts/game/prop_spawner.gd scenes/props tests/
git commit -m "feat: seeded prop layout generator and crate spawner

Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>"
```

---

### Task 6: Blackboard and lidar sensor

**Files:**
- Create: `scripts/robot/perception.gd`, `scripts/robot/sensors/sensor_base.gd`, `scripts/robot/sensors/lidar.gd`
- Modify: `scenes/robots/robot.tscn` (add `Sensors/Lidar` node)
- Test: `tests/unit/blackboard_test.gd`, `tests/scene/lidar_test.gd`

**Interfaces:**
- Produces:
  - `class_name Blackboard extends RefCounted`
    - inputs written by the robot each tick: `time: float`, `position: Vector3`, `yaw: float`, `rng: RandomNumberGenerator`, `role: String`, `caught_ids: Array[int]`
    - lidar: `lidar_enabled: bool`, `lidar_ranges: PackedFloat32Array`, `lidar_angles: PackedFloat32Array` (robot-relative radians, 0 = forward, +CCW), `lidar_max_range: float`
    - vision: `vision_enabled: bool`, `seen_robots: Array[Dictionary]` `{id, role, position, bearing, distance, time}`; memory `last_known: Dictionary` id → `{position, time, role}`; `seeker_last_seen_pos: Vector3`, `seeker_last_seen_time: float = -INF`
    - ultrasonic: `ultrasonic_enabled: bool`, `ultrasonic: PackedFloat32Array` `[front, left, right]`
    - bumper: `bumper_enabled: bool`, `bumper_hit: bool`, `bumper_normal: Vector3`
    - blobs: `blobs: Array[Dictionary]` `{position: Vector3, distance, angle, time}` (filled in Task 7)
    - motion feedback: `stuck: bool`, `goal_reached: bool`
    - FSM outputs: `goal_pos: Vector3`, `goal_active: bool`, `catch_request: int = 0`, `state_name: String`
    - helpers: `func nearest_obstacle() -> Dictionary` `{distance, angle}` (INF if none), `func min_range_in_arc(center: float, half_width: float) -> float`, `func time_since_seeker_seen() -> float`, `func seen_hiders() -> Array[Dictionary]` (uncaught, role == "hider"), `func seen_seeker() -> Dictionary` (empty if none), `func local_to_world_dir(angle: float) -> Vector3`, `func world_to_local_angle(p: Vector3) -> float`
  - `class_name SensorBase extends Node3D`: `@export var enabled: bool = true`, `@export var noise_std: float = 0.0`, `var body: DriveBody` (set by robot), `func sense(bb: Blackboard) -> void` (abstract, override), `func _noise(bb: Blackboard) -> float` returns `bb.rng.randfn(0.0, noise_std)` or 0.
  - `class_name Lidar extends SensorBase`: `@export var ray_count: int = 36`, `@export var max_range: float = 8.0`, `@export var collision_mask: int = 7`. Writes `lidar_*` fields. Unhit rays read `max_range`.

- [ ] **Step 1: Write failing blackboard unit tests**

`tests/unit/blackboard_test.gd`:

```gdscript
extends GdUnitTestSuite


func _bb_with_lidar(ranges: Array) -> Blackboard:
	var bb := Blackboard.new()
	bb.lidar_enabled = true
	bb.lidar_max_range = 8.0
	var n: int = ranges.size()
	for i in n:
		bb.lidar_ranges.append(ranges[i])
		bb.lidar_angles.append(wrapf(TAU * i / n, -PI, PI))
	return bb


func test_nearest_obstacle_picks_min_range() -> void:
	var bb := _bb_with_lidar([8, 8, 2.5, 8, 8, 8, 8, 8])
	var o: Dictionary = bb.nearest_obstacle()
	assert_float(o["distance"]).is_equal_approx(2.5, 1e-6)
	assert_float(o["angle"]).is_equal_approx(TAU * 2 / 8, 1e-6)


func test_nearest_obstacle_inf_when_nothing_hit() -> void:
	var bb := _bb_with_lidar([8, 8, 8, 8])
	assert_bool(is_inf(bb.nearest_obstacle()["distance"])).is_true()


func test_min_range_in_arc_front() -> void:
	# angles: 0, 45, 90, 135, 180, -135, -90, -45 deg
	var bb := _bb_with_lidar([5, 8, 8, 8, 1, 8, 8, 3])
	assert_float(bb.min_range_in_arc(0.0, deg_to_rad(50))).is_equal_approx(3.0, 1e-6)
	assert_float(bb.min_range_in_arc(PI, deg_to_rad(10))).is_equal_approx(1.0, 1e-6)


func test_local_world_angle_roundtrip() -> void:
	var bb := Blackboard.new()
	bb.position = Vector3(1, 0, 2)
	bb.yaw = 0.3
	var dir: Vector3 = bb.local_to_world_dir(0.5)
	var p: Vector3 = bb.position + dir * 3.0
	assert_float(bb.world_to_local_angle(p)).is_equal_approx(0.5, 1e-5)


func test_forward_is_minus_z_at_zero_yaw() -> void:
	var bb := Blackboard.new()
	assert_vector(bb.local_to_world_dir(0.0)).is_equal_approx(Vector3(0, 0, -1), Vector3(1e-6, 1e-6, 1e-6))
	# positive angle turns left (toward -X)
	assert_float(bb.local_to_world_dir(0.5).x).is_less(0.0)


func test_seen_hiders_excludes_caught_and_seekers() -> void:
	var bb := Blackboard.new()
	bb.caught_ids = [2]
	bb.seen_robots = [
		{"id": 1, "role": "hider", "position": Vector3.ZERO, "bearing": 0.0, "distance": 1.0, "time": 0.0},
		{"id": 2, "role": "hider", "position": Vector3.ZERO, "bearing": 0.0, "distance": 1.0, "time": 0.0},
		{"id": 3, "role": "seeker", "position": Vector3.ZERO, "bearing": 0.0, "distance": 1.0, "time": 0.0},
	]
	var h := bb.seen_hiders()
	assert_int(h.size()).is_equal(1)
	assert_int(h[0]["id"]).is_equal(1)
	assert_int(bb.seen_seeker()["id"]).is_equal(3)


func test_time_since_seeker_seen() -> void:
	var bb := Blackboard.new()
	bb.time = 10.0
	assert_bool(is_inf(bb.time_since_seeker_seen())).is_true()
	bb.seeker_last_seen_time = 7.5
	assert_float(bb.time_since_seeker_seen()).is_equal_approx(2.5, 1e-6)
```

- [ ] **Step 2: Run, expect failure**

Run: `./run_tests.sh` → `Blackboard` unknown.

- [ ] **Step 3: Implement perception.gd**

`scripts/robot/perception.gd`:

```gdscript
## Per-robot blackboard: everything the decision and motion layers may read.
## Sensors write here; nothing else reads the scene tree.
class_name Blackboard
extends RefCounted

# --- robot state (written by RobotBrain each tick)
var time: float = 0.0
var position: Vector3 = Vector3.ZERO
var yaw: float = 0.0
var rng: RandomNumberGenerator = RandomNumberGenerator.new()
var role: String = "hider"
var caught_ids: Array[int] = []

# --- lidar
var lidar_enabled: bool = false
var lidar_ranges: PackedFloat32Array = PackedFloat32Array()
var lidar_angles: PackedFloat32Array = PackedFloat32Array()
var lidar_max_range: float = 8.0

# --- vision
var vision_enabled: bool = false
var seen_robots: Array[Dictionary] = []
var last_known: Dictionary = {}  # id -> {position, time, role}
var seeker_last_seen_pos: Vector3 = Vector3.ZERO
var seeker_last_seen_time: float = -INF

# --- ultrasonic / bumper
var ultrasonic_enabled: bool = false
var ultrasonic: PackedFloat32Array = PackedFloat32Array([INF, INF, INF])
var bumper_enabled: bool = false
var bumper_hit: bool = false
var bumper_normal: Vector3 = Vector3.ZERO

# --- derived
var blobs: Array[Dictionary] = []

# --- motion feedback
var stuck: bool = false
var goal_reached: bool = false

# --- FSM outputs
var goal_pos: Vector3 = Vector3.ZERO
var goal_active: bool = false
var catch_request: int = 0
var state_name: String = ""


func local_to_world_dir(angle: float) -> Vector3:
	var a: float = yaw + angle
	return Vector3(-sin(a), 0.0, -cos(a))


func world_to_local_angle(p: Vector3) -> float:
	var d: Vector3 = p - position
	var world_angle: float = atan2(-d.x, -d.z)
	return wrapf(world_angle - yaw, -PI, PI)


func nearest_obstacle() -> Dictionary:
	var best_d: float = INF
	var best_a: float = 0.0
	for i in lidar_ranges.size():
		if lidar_ranges[i] < lidar_max_range - 1e-3 and lidar_ranges[i] < best_d:
			best_d = lidar_ranges[i]
			best_a = lidar_angles[i]
	return {"distance": best_d, "angle": best_a}


func min_range_in_arc(center: float, half_width: float) -> float:
	var best: float = INF
	for i in lidar_ranges.size():
		if absf(wrapf(lidar_angles[i] - center, -PI, PI)) <= half_width + 1e-6:
			best = minf(best, lidar_ranges[i])
	return best


func time_since_seeker_seen() -> float:
	return time - seeker_last_seen_time


func seen_hiders() -> Array[Dictionary]:
	var out: Array[Dictionary] = []
	for s in seen_robots:
		if s["role"] == "hider" and not caught_ids.has(s["id"]):
			out.append(s)
	return out


func seen_seeker() -> Dictionary:
	for s in seen_robots:
		if s["role"] == "seeker":
			return s
	return {}
```

- [ ] **Step 4: Run, expect blackboard tests pass**

Run: `./run_tests.sh`.

- [ ] **Step 5: Write the failing lidar scene test**

`tests/scene/lidar_test.gd`:

```gdscript
extends GdUnitTestSuite

const ROBOT := preload("res://scenes/robots/robot.tscn")
const FLOOR := preload("res://tests/fixtures/flat_floor.tscn")


func test_lidar_reads_wall_distance_and_max_range_elsewhere() -> void:
	var runner := scene_runner(FLOOR.instantiate())
	var robot: DriveBody = ROBOT.instantiate()
	robot.position = Vector3(0, 0.3, 0)
	runner.scene().add_child(robot)
	await runner.simulate_frames(5)
	var lidar: Lidar = robot.get_node("Sensors/Lidar")
	lidar.body = robot
	var bb := Blackboard.new()
	bb.position = robot.global_position
	bb.yaw = robot.yaw()
	lidar.sense(bb)
	assert_bool(bb.lidar_enabled).is_true()
	assert_int(bb.lidar_ranges.size()).is_equal(36)
	# Wall at x=+5 is to the robot's right (-90 deg) when facing -Z.
	var right_idx: int = -1
	for i in bb.lidar_angles.size():
		if absf(wrapf(bb.lidar_angles[i] + PI / 2, -PI, PI)) < 1e-4:
			right_idx = i
	assert_int(right_idx).is_not_equal(-1)
	assert_float(bb.lidar_ranges[right_idx]).is_equal_approx(4.9, 0.1)  # wall face: centre 5.0 minus half thickness
	# Forward (-Z) is open for 30 m: reads max_range.
	assert_float(bb.lidar_ranges[0]).is_equal_approx(lidar.max_range, 1e-3)


func test_disabled_lidar_leaves_blackboard_empty() -> void:
	var runner := scene_runner(FLOOR.instantiate())
	var robot: DriveBody = ROBOT.instantiate()
	runner.scene().add_child(robot)
	await runner.simulate_frames(2)
	var lidar: Lidar = robot.get_node("Sensors/Lidar")
	lidar.body = robot
	lidar.enabled = false
	var bb := Blackboard.new()
	lidar.sense(bb)
	assert_bool(bb.lidar_enabled).is_false()
	assert_int(bb.lidar_ranges.size()).is_equal(0)
```

- [ ] **Step 6: Run, expect failure**

Run: `./run_tests.sh` → node `Sensors/Lidar` missing.

- [ ] **Step 7: Implement sensor_base.gd and lidar.gd, add node to robot.tscn**

`scripts/robot/sensors/sensor_base.gd`:

```gdscript
## Base for simulated sensors. Subclasses override sense(bb).
class_name SensorBase
extends Node3D

@export var enabled: bool = true
@export var noise_std: float = 0.0

var body: DriveBody


func sense(_bb: Blackboard) -> void:
	pass


func _noise(bb: Blackboard) -> float:
	if noise_std <= 0.0:
		return 0.0
	return bb.rng.randfn(0.0, noise_std)


func _space() -> PhysicsDirectSpaceState3D:
	return get_world_3d().direct_space_state
```

`scripts/robot/sensors/lidar.gd`:

```gdscript
## 2D lidar: a horizontal fan of raycasts. Unhit rays report max_range.
class_name Lidar
extends SensorBase

@export var ray_count: int = 36
@export var max_range: float = 8.0
@export var collision_mask: int = 7


func sense(bb: Blackboard) -> void:
	bb.lidar_enabled = enabled
	bb.lidar_ranges.clear()
	bb.lidar_angles.clear()
	bb.lidar_max_range = max_range
	if not enabled:
		return
	var origin: Vector3 = global_position
	var space := _space()
	var exclude: Array[RID] = [body.get_rid()]
	for i in ray_count:
		var angle: float = wrapf(TAU * i / ray_count, -PI, PI)
		var dir: Vector3 = bb.local_to_world_dir(angle)
		var q := PhysicsRayQueryParameters3D.create(origin, origin + dir * max_range, collision_mask, exclude)
		var hit: Dictionary = space.intersect_ray(q)
		var r: float = max_range
		if not hit.is_empty():
			r = minf(max_range, origin.distance_to(hit["position"]) + _noise(bb))
		bb.lidar_ranges.append(r)
		bb.lidar_angles.append(angle)
```

Add to `robot.tscn` (after the `Sensors` node):

```
[ext_resource type="Script" path="res://scripts/robot/sensors/lidar.gd" id="lidar"]

[node name="Lidar" type="Node3D" parent="Sensors"]
script = ExtResource("lidar")
```

(Update `load_steps` accordingly: it equals the number of ext + sub resources + 1.)

- [ ] **Step 8: Run, expect pass**

Run: `./run_tests.sh`.

- [ ] **Step 9: Commit**

```bash
git add scripts/robot/perception.gd scripts/robot/sensors scenes/robots/robot.tscn tests/
git commit -m "feat: blackboard and lidar sensor

Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>"
```

---

### Task 7: Lidar blob detection and vision cone

**Files:**
- Create: `scripts/robot/lidar_blobs.gd`, `scripts/robot/sensors/vision_cone.gd`
- Modify: `scenes/robots/robot.tscn` (add `Sensors/Vision`)
- Test: `tests/unit/lidar_blobs_test.gd`, `tests/scene/vision_cone_test.gd`

**Interfaces:**
- Produces:
  - `class_name LidarBlobs`: `static func detect(ranges: PackedFloat32Array, angles: PackedFloat32Array, max_range: float, min_width: float = 0.2, max_width: float = 0.8, break_gap: float = 0.5) -> Array[Dictionary]` each `{"angle": float, "distance": float, "width": float}` (robot-relative). Clusters consecutive hit rays whose range differs by < `break_gap`; keeps clusters whose chord width (distance between first and last hit points) is within [min, max] AND whose neighbours on both sides are either unhit or farther by > `break_gap` (i.e. the blob stands free of walls).
  - `class_name VisionCone extends SensorBase`: `@export var fov_deg: float = 90.0`, `@export var max_range: float = 10.0`, `@export var los_mask: int = 7`. For each `DriveBody` in group `robots` (not self): if within range and |bearing| ≤ fov/2 and a LOS ray from eye to the target's position + (0, 0.15, 0) hits that body first → append to `bb.seen_robots`, update `bb.last_known[id]`, and if role == "seeker" set `bb.seeker_last_seen_pos/time`.

- [ ] **Step 1: Write failing blob tests**

`tests/unit/lidar_blobs_test.gd`:

```gdscript
extends GdUnitTestSuite

const N := 72  # 5 degree resolution


func _scan(hits: Dictionary) -> Array:  # idx -> range ; returns [ranges, angles]
	var ranges := PackedFloat32Array()
	var angles := PackedFloat32Array()
	for i in N:
		ranges.append(hits.get(i, 8.0))
		angles.append(wrapf(TAU * i / N, -PI, PI))
	return [ranges, angles]


func test_robot_sized_cluster_is_a_blob() -> void:
	# 3 consecutive rays at 3 m: chord ~= 3 * 2 * sin(5deg) = 0.52 m
	var s := _scan({10: 3.0, 11: 3.0, 12: 3.0})
	var blobs := LidarBlobs.detect(s[0], s[1], 8.0)
	assert_int(blobs.size()).is_equal(1)
	assert_float(blobs[0]["distance"]).is_equal_approx(3.0, 1e-3)
	assert_float(blobs[0]["angle"]).is_equal_approx(TAU * 11 / N, 1e-6)
	assert_float(blobs[0]["width"]).is_between(0.4, 0.6)


func test_long_wall_is_not_a_blob() -> void:
	var hits := {}
	for i in range(20, 40):
		hits[i] = 4.0
	var s := _scan(hits)
	assert_array(LidarBlobs.detect(s[0], s[1], 8.0)).is_empty()


func test_cluster_touching_wall_is_not_a_blob() -> void:
	# wall at 4 m from idx 20..40, then an object at 3.8 m right next to it (continuous)
	var hits := {}
	for i in range(20, 40):
		hits[i] = 4.0
	hits[40] = 3.8
	hits[41] = 3.8
	var s := _scan(hits)
	assert_array(LidarBlobs.detect(s[0], s[1], 8.0)).is_empty()


func test_object_in_front_of_wall_is_a_blob_if_gap_is_large() -> void:
	var hits := {}
	for i in range(20, 50):
		hits[i] = 6.0
	hits[30] = 2.0
	hits[31] = 2.0
	hits[32] = 2.0
	var s := _scan(hits)
	var blobs := LidarBlobs.detect(s[0], s[1], 8.0)
	assert_int(blobs.size()).is_equal(1)
	assert_float(blobs[0]["distance"]).is_equal_approx(2.0, 1e-3)


func test_wraparound_cluster_across_index_zero() -> void:
	var s := _scan({71: 2.5, 0: 2.5, 1: 2.5})
	var blobs := LidarBlobs.detect(s[0], s[1], 8.0)
	assert_int(blobs.size()).is_equal(1)
	assert_float(absf(blobs[0]["angle"])).is_less(0.01)


func test_single_ray_too_narrow() -> void:
	var s := _scan({10: 1.0})  # chord 0 m
	assert_array(LidarBlobs.detect(s[0], s[1], 8.0)).is_empty()
```

- [ ] **Step 2: Run, expect failure**

Run: `./run_tests.sh` → `LidarBlobs` unknown.

- [ ] **Step 3: Implement lidar_blobs.gd**

`scripts/robot/lidar_blobs.gd`:

```gdscript
## Clusters lidar hits into free-standing, robot-sized blobs.
## This is the seeker's fallback detector when vision is disabled.
class_name LidarBlobs
extends RefCounted


static func detect(ranges: PackedFloat32Array, angles: PackedFloat32Array, max_range: float,
		min_width: float = 0.2, max_width: float = 0.8, break_gap: float = 0.5) -> Array[Dictionary]:
	var out: Array[Dictionary] = []
	var n: int = ranges.size()
	if n < 3:
		return out
	var hit := func(i: int) -> bool: return ranges[posmod(i, n)] < max_range - 1e-3
	# Find a start index that is not inside a cluster, to handle wraparound.
	var start: int = -1
	for i in n:
		if not hit.call(i):
			start = i
			break
	if start == -1:
		return out  # everything hit: one giant ring, no blobs
	var i: int = start
	var visited: int = 0
	while visited < n:
		var k: int = posmod(i, n)
		if not hit.call(k):
			i += 1
			visited += 1
			continue
		# cluster from k while consecutive range differences are small
		var idxs: Array[int] = [k]
		var j: int = i + 1
		while visited + idxs.size() < n and hit.call(j) and absf(ranges[posmod(j, n)] - ranges[posmod(j - 1, n)]) < break_gap:
			idxs.append(posmod(j, n))
			j += 1
		visited += idxs.size()
		i = j
		# free-standing check: neighbours either unhit or far behind
		var before: int = posmod(idxs[0] - 1, n)
		var after: int = posmod(idxs[-1] + 1, n)
		var first_r: float = ranges[idxs[0]]
		var last_r: float = ranges[idxs[-1]]
		var free_before: bool = (not hit.call(before)) or ranges[before] - first_r > break_gap
		var free_after: bool = (not hit.call(after)) or ranges[after] - last_r > break_gap
		if not (free_before and free_after):
			continue
		var p0 := Vector2(first_r * cos(angles[idxs[0]]), first_r * sin(angles[idxs[0]]))
		var p1 := Vector2(last_r * cos(angles[idxs[-1]]), last_r * sin(angles[idxs[-1]]))
		var width: float = p0.distance_to(p1)
		if width < min_width or width > max_width:
			continue
		var sum_d: float = 0.0
		var sx: float = 0.0
		var sy: float = 0.0
		for idx in idxs:
			sum_d += ranges[idx]
			sx += cos(angles[idx])
			sy += sin(angles[idx])
		out.append({
			"angle": atan2(sy, sx),
			"distance": sum_d / idxs.size(),
			"width": width,
		})
	return out
```

- [ ] **Step 4: Run, expect blob tests pass**

Run: `./run_tests.sh`.

- [ ] **Step 5: Write the failing vision scene test**

`tests/scene/vision_cone_test.gd`:

```gdscript
extends GdUnitTestSuite

const ROBOT := preload("res://scenes/robots/robot.tscn")
const FLOOR := preload("res://tests/fixtures/flat_floor.tscn")


func _spawn(runner: GdUnitSceneRunner, pos: Vector3, role: String, yaw: float = 0.0) -> DriveBody:
	var r: DriveBody = ROBOT.instantiate()
	r.position = pos
	r.rotation.y = yaw
	r.role = role
	runner.scene().add_child(r)
	return r


func _sense(seeker: DriveBody) -> Blackboard:
	var vision: VisionCone = seeker.get_node("Sensors/Vision")
	vision.body = seeker
	var bb := Blackboard.new()
	bb.time = 4.0
	bb.position = seeker.global_position
	bb.yaw = seeker.yaw()
	vision.sense(bb)
	return bb


func test_sees_robot_in_front_within_range() -> void:
	var runner := scene_runner(FLOOR.instantiate())
	var seeker := _spawn(runner, Vector3(0, 0.3, 0), "seeker")
	var hider := _spawn(runner, Vector3(0.5, 0.3, -6), "hider")
	await runner.simulate_frames(3)
	var bb := _sense(seeker)
	assert_int(bb.seen_robots.size()).is_equal(1)
	var s: Dictionary = bb.seen_robots[0]
	assert_int(s["id"]).is_equal(hider.get_instance_id())
	assert_str(s["role"]).is_equal("hider")
	assert_float(s["distance"]).is_equal_approx(6.02, 0.1)
	assert_float(s["bearing"]).is_less(0.0)  # slightly to the right (+X)
	assert_bool(bb.last_known.has(hider.get_instance_id())).is_true()


func test_does_not_see_robot_behind() -> void:
	var runner := scene_runner(FLOOR.instantiate())
	var seeker := _spawn(runner, Vector3(0, 0.3, 0), "seeker")
	_spawn(runner, Vector3(0, 0.3, 6), "hider")
	await runner.simulate_frames(3)
	assert_array(_sense(seeker).seen_robots).is_empty()


func test_does_not_see_robot_beyond_range() -> void:
	var runner := scene_runner(FLOOR.instantiate())
	var seeker := _spawn(runner, Vector3(0, 0.3, 0), "seeker")
	_spawn(runner, Vector3(0, 0.3, -12), "hider")
	await runner.simulate_frames(3)
	assert_array(_sense(seeker).seen_robots).is_empty()


func test_wall_blocks_line_of_sight() -> void:
	var runner := scene_runner(FLOOR.instantiate())
	# Seeker faces +X (yaw -90deg) toward the wall at x=5; hider behind it at x=7.
	var seeker := _spawn(runner, Vector3(2, 0.3, 0), "seeker", -PI / 2)
	_spawn(runner, Vector3(7, 0.3, 0), "hider")
	await runner.simulate_frames(3)
	assert_array(_sense(seeker).seen_robots).is_empty()


func test_seeing_seeker_updates_seeker_memory() -> void:
	var runner := scene_runner(FLOOR.instantiate())
	var hider := _spawn(runner, Vector3(0, 0.3, 0), "hider")
	var seeker := _spawn(runner, Vector3(0, 0.3, -4), "seeker")
	await runner.simulate_frames(3)
	var bb := _sense(hider)
	assert_float(bb.seeker_last_seen_time).is_equal_approx(4.0, 1e-6)
	assert_float(bb.seeker_last_seen_pos.distance_to(seeker.global_position)).is_less(0.01)


func test_disabled_vision_sees_nothing() -> void:
	var runner := scene_runner(FLOOR.instantiate())
	var seeker := _spawn(runner, Vector3(0, 0.3, 0), "seeker")
	_spawn(runner, Vector3(0, 0.3, -3), "hider")
	await runner.simulate_frames(3)
	(seeker.get_node("Sensors/Vision") as VisionCone).enabled = false
	var bb := _sense(seeker)
	assert_bool(bb.vision_enabled).is_false()
	assert_array(bb.seen_robots).is_empty()
```

- [ ] **Step 6: Run, expect failure**

Run: `./run_tests.sh` → `Sensors/Vision` missing.

- [ ] **Step 7: Implement vision_cone.gd and add node**

`scripts/robot/sensors/vision_cone.gd`:

```gdscript
## Camera-like detector: limited FOV and range, confirmed by a line-of-sight ray.
class_name VisionCone
extends SensorBase

@export var fov_deg: float = 90.0
@export var max_range: float = 10.0
@export var los_mask: int = 7


func sense(bb: Blackboard) -> void:
	bb.vision_enabled = enabled
	bb.seen_robots.clear()
	if not enabled:
		return
	var eye: Vector3 = global_position
	var half_fov: float = deg_to_rad(fov_deg) * 0.5
	var space := _space()
	var exclude: Array[RID] = [body.get_rid()]
	for node in get_tree().get_nodes_in_group("robots"):
		var other := node as DriveBody
		if other == null or other == body:
			continue
		var target: Vector3 = other.global_position + Vector3(0, 0.15, 0)
		var flat: Vector3 = other.global_position - bb.position  # centre to centre, like the tag range
		flat.y = 0.0
		var dist: float = flat.length()
		if dist > max_range:
			continue
		var bearing: float = bb.world_to_local_angle(other.global_position)
		if absf(bearing) > half_fov:
			continue
		var q := PhysicsRayQueryParameters3D.create(eye, target, los_mask, exclude)
		var hit: Dictionary = space.intersect_ray(q)
		if hit.is_empty() or hit["collider"] != other:
			continue
		var id: int = other.get_instance_id()
		var rec := {
			"id": id,
			"role": other.role,
			"position": other.global_position,
			"bearing": bearing + _noise(bb),
			"distance": dist + _noise(bb),
			"time": bb.time,
		}
		bb.seen_robots.append(rec)
		bb.last_known[id] = {"position": other.global_position, "time": bb.time, "role": other.role}
		if other.role == "seeker":
			bb.seeker_last_seen_pos = other.global_position
			bb.seeker_last_seen_time = bb.time
```

Add to `robot.tscn`:

```
[ext_resource type="Script" path="res://scripts/robot/sensors/vision_cone.gd" id="vision"]

[node name="Vision" type="Node3D" parent="Sensors"]
transform = Transform3D(1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0.1, -0.2)
script = ExtResource("vision")
```

- [ ] **Step 8: Run, expect pass**

Run: `./run_tests.sh`. The robots group test uses `get_tree()`, so the sensor must be inside the tree (it is, as a child of the robot).

- [ ] **Step 9: Commit**

```bash
git add scripts/robot/lidar_blobs.gd scripts/robot/sensors/vision_cone.gd scenes/robots/robot.tscn tests/
git commit -m "feat: lidar blob detection and vision cone sensor

Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>"
```

---

### Task 8: Ultrasonic and bumper sensors

**Files:**
- Create: `scripts/robot/sensors/ultrasonic.gd`, `scripts/robot/sensors/bumper.gd`
- Modify: `scenes/robots/robot.tscn` (add `Sensors/Ultrasonic`, `Sensors/Bumper`)
- Test: `tests/scene/close_range_sensors_test.gd`

**Interfaces:**
- Produces:
  - `class_name Ultrasonic extends SensorBase`: `@export var max_range: float = 1.2`, `@export var cone_half_deg: float = 15.0`, `@export var rays_per_cone: int = 3`, `@export var directions_deg: PackedFloat32Array = [0.0, 35.0, -35.0]` (front, left, right), `@export var collision_mask: int = 7`. Writes `bb.ultrasonic = [front, left, right]`, `max_range` when nothing hit.
  - `class_name Bumper extends SensorBase`: `@export var front_half_deg: float = 70.0`. Reads `body.contacts`; a contact whose position is in the front arc (local angle of the contact position within ±70°) sets `bb.bumper_hit = true` and `bb.bumper_normal` to that contact's normal.

- [ ] **Step 1: Write the failing scene test**

`tests/scene/close_range_sensors_test.gd`:

```gdscript
extends GdUnitTestSuite

const ROBOT := preload("res://scenes/robots/robot.tscn")
const FLOOR := preload("res://tests/fixtures/flat_floor.tscn")
const CRATE := preload("res://scenes/props/crate.tscn")


func _robot(runner: GdUnitSceneRunner, pos: Vector3, yaw: float) -> DriveBody:
	var r: DriveBody = ROBOT.instantiate()
	r.position = pos
	r.rotation.y = yaw
	runner.scene().add_child(r)
	for s in r.get_node("Sensors").get_children():
		if s is SensorBase:
			(s as SensorBase).body = r
	return r


func _bb(r: DriveBody) -> Blackboard:
	var bb := Blackboard.new()
	bb.position = r.global_position
	bb.yaw = r.yaw()
	return bb


func test_ultrasonic_front_reads_crate_distance() -> void:
	var runner := scene_runner(FLOOR.instantiate())
	var crate: StaticBody3D = CRATE.instantiate()
	crate.position = Vector3(0, 0.5, -1.5)  # face at z=-1.0
	runner.scene().add_child(crate)
	var r := _robot(runner, Vector3(0, 0.3, 0), 0.0)
	await runner.simulate_frames(3)
	var bb := _bb(r)
	(r.get_node("Sensors/Ultrasonic") as Ultrasonic).sense(bb)
	assert_bool(bb.ultrasonic_enabled).is_true()
	# sensor sits ~0.15 m ahead of the body origin, face at 1.0 => ~0.85
	assert_float(bb.ultrasonic[0]).is_between(0.7, 1.0)
	assert_float(bb.ultrasonic[1]).is_equal_approx(1.2, 1e-3)
	assert_float(bb.ultrasonic[2]).is_equal_approx(1.2, 1e-3)


func test_ultrasonic_side_cones() -> void:
	var runner := scene_runner(FLOOR.instantiate())
	var crate: StaticBody3D = CRATE.instantiate()
	crate.position = Vector3(-0.8, 0.5, -0.9)  # ahead-left
	runner.scene().add_child(crate)
	var r := _robot(runner, Vector3(0, 0.3, 0), 0.0)
	await runner.simulate_frames(3)
	var bb := _bb(r)
	(r.get_node("Sensors/Ultrasonic") as Ultrasonic).sense(bb)
	assert_float(bb.ultrasonic[1]).is_less(1.2)       # left sees it
	assert_float(bb.ultrasonic[2]).is_equal_approx(1.2, 1e-3)  # right does not


func test_bumper_fires_on_frontal_wall_contact_only() -> void:
	var runner := scene_runner(FLOOR.instantiate())
	var r := _robot(runner, Vector3(4.0, 0.3, 0), -PI / 2)  # facing +X toward wall at x=5
	await runner.simulate_frames(5)
	var bb := _bb(r)
	(r.get_node("Sensors/Bumper") as Bumper).sense(bb)
	assert_bool(bb.bumper_hit).is_false()
	r.set_command(1.0, 0.0)
	await runner.simulate_frames(90)
	bb = _bb(r)
	(r.get_node("Sensors/Bumper") as Bumper).sense(bb)
	assert_bool(bb.bumper_hit).is_true()
	assert_float(bb.bumper_normal.x).is_less(-0.9)


func test_bumper_ignores_rear_contact() -> void:
	var runner := scene_runner(FLOOR.instantiate())
	var r := _robot(runner, Vector3(4.0, 0.3, 0), PI / 2)  # facing -X, wall is behind
	await runner.simulate_frames(5)
	r.set_command(-1.0, 0.0)  # reverse into the wall
	await runner.simulate_frames(90)
	var bb := _bb(r)
	(r.get_node("Sensors/Bumper") as Bumper).sense(bb)
	assert_bool(bb.bumper_hit).is_false()
	assert_array(r.contacts).is_not_empty()  # we did touch it
```

- [ ] **Step 2: Run, expect failure**

- [ ] **Step 3: Implement**

`scripts/robot/sensors/ultrasonic.gd`:

```gdscript
## Three short-range proximity cones (front, left, right). Each cone is a few
## rays; the minimum hit distance is reported, max_range when clear.
class_name Ultrasonic
extends SensorBase

@export var max_range: float = 1.2
@export var cone_half_deg: float = 15.0
@export var rays_per_cone: int = 3
@export var directions_deg: PackedFloat32Array = PackedFloat32Array([0.0, 35.0, -35.0])
@export var collision_mask: int = 7


func sense(bb: Blackboard) -> void:
	bb.ultrasonic_enabled = enabled
	bb.ultrasonic = PackedFloat32Array([max_range, max_range, max_range])
	if not enabled:
		return
	var origin: Vector3 = global_position
	var space := _space()
	var exclude: Array[RID] = [body.get_rid()]
	for c in directions_deg.size():
		var center: float = deg_to_rad(directions_deg[c])
		var best: float = max_range
		for k in rays_per_cone:
			var t: float = -1.0 + 2.0 * k / maxi(1, rays_per_cone - 1) if rays_per_cone > 1 else 0.0
			var angle: float = center + t * deg_to_rad(cone_half_deg)
			var dir: Vector3 = bb.local_to_world_dir(angle)
			var q := PhysicsRayQueryParameters3D.create(origin, origin + dir * max_range, collision_mask, exclude)
			var hit: Dictionary = space.intersect_ray(q)
			if not hit.is_empty():
				best = minf(best, origin.distance_to(hit["position"]) + _noise(bb))
		bb.ultrasonic[c] = clampf(best, 0.0, max_range)
```

`scripts/robot/sensors/bumper.gd`:

```gdscript
## Front-arc contact sensor fed by DriveBody.contacts.
class_name Bumper
extends SensorBase

@export var front_half_deg: float = 70.0


func sense(bb: Blackboard) -> void:
	bb.bumper_enabled = enabled
	bb.bumper_hit = false
	bb.bumper_normal = Vector3.ZERO
	if not enabled:
		return
	var half: float = deg_to_rad(front_half_deg)
	for c in body.contacts:
		var angle: float = bb.world_to_local_angle(c["position"])
		if absf(angle) <= half:
			bb.bumper_hit = true
			bb.bumper_normal = c["normal"]
			return
```

Add to `robot.tscn`:

```
[ext_resource type="Script" path="res://scripts/robot/sensors/ultrasonic.gd" id="ultrasonic"]
[ext_resource type="Script" path="res://scripts/robot/sensors/bumper.gd" id="bumper"]

[node name="Ultrasonic" type="Node3D" parent="Sensors"]
transform = Transform3D(1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, -0.15)
script = ExtResource("ultrasonic")

[node name="Bumper" type="Node3D" parent="Sensors"]
script = ExtResource("bumper")
```

- [ ] **Step 4: Run, expect pass**

Run: `./run_tests.sh`. If the bumper normal sign in `test_bumper_fires_on_frontal_wall_contact_only` disagrees with Task 3's wall test, the convention is wrong in `drive.gd`, not here: fix it there so both tests pass.

- [ ] **Step 5: Commit**

```bash
git add scripts/robot/sensors scenes/robots/robot.tscn tests/
git commit -m "feat: ultrasonic and bumper sensors

Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>"
```

---

### Task 9: Sensor debug overlay

**Files:**
- Create: `scripts/robot/sensors/debug_overlay.gd`
- Modify: `scenes/robots/robot.tscn` (add `DebugOverlay` MeshInstance3D child of root)

**Interfaces:**
- Produces: `class_name DebugOverlay extends MeshInstance3D`, `func draw(bb: Blackboard, lidar: Lidar, vision: VisionCone) -> void`, `static var visible_globally: bool = false` toggled by the HUD (Task 16). When hidden, `draw` clears the mesh and returns.

No unit test: this is pure rendering. Verification is visual in Step 3.

- [ ] **Step 1: Implement**

`scripts/robot/sensors/debug_overlay.gd`:

```gdscript
## Draws lidar rays and the vision cone with an ImmediateMesh. Toggle with
## DebugOverlay.visible_globally (HUD key F1). Draw happens in robot-local space.
class_name DebugOverlay
extends MeshInstance3D

static var visible_globally: bool = false

var _im := ImmediateMesh.new()


func _ready() -> void:
	mesh = _im
	var mat := StandardMaterial3D.new()
	mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	mat.vertex_color_use_as_albedo = true
	mat.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	material_override = mat
	top_level = true  # we draw in world space


func draw(bb: Blackboard, lidar: Lidar, vision: VisionCone) -> void:
	_im.clear_surfaces()
	if not visible_globally:
		return
	global_transform = Transform3D.IDENTITY
	var origin: Vector3 = bb.position + Vector3(0, 0.15, 0)
	if bb.lidar_enabled and bb.lidar_ranges.size() > 0:
		_im.surface_begin(Mesh.PRIMITIVE_LINES)
		for i in bb.lidar_ranges.size():
			var r: float = bb.lidar_ranges[i]
			var hit: bool = r < bb.lidar_max_range - 1e-3
			_im.surface_set_color(Color(1, 0.3, 0.2, 0.9) if hit else Color(0.4, 0.9, 0.4, 0.25))
			_im.surface_add_vertex(origin)
			_im.surface_add_vertex(origin + bb.local_to_world_dir(bb.lidar_angles[i]) * r)
		_im.surface_end()
	if bb.vision_enabled and vision != null:
		var half: float = deg_to_rad(vision.fov_deg) * 0.5
		var col := Color(0.3, 0.6, 1.0, 0.8) if bb.seen_robots.is_empty() else Color(1.0, 0.9, 0.2, 0.9)
		_im.surface_begin(Mesh.PRIMITIVE_LINE_STRIP)
		_im.surface_set_color(col)
		_im.surface_add_vertex(origin)
		for k in 17:
			var a: float = -half + 2.0 * half * k / 16.0
			_im.surface_add_vertex(origin + bb.local_to_world_dir(a) * vision.max_range)
		_im.surface_add_vertex(origin)
		_im.surface_end()
	if bb.goal_active:
		_im.surface_begin(Mesh.PRIMITIVE_LINES)
		_im.surface_set_color(Color(1, 1, 1, 0.8))
		_im.surface_add_vertex(origin)
		_im.surface_add_vertex(bb.goal_pos + Vector3(0, 0.15, 0))
		_im.surface_end()
```

Add to `robot.tscn`:

```
[ext_resource type="Script" path="res://scripts/robot/sensors/debug_overlay.gd" id="overlay"]

[node name="DebugOverlay" type="MeshInstance3D" parent="."]
script = ExtResource("overlay")
```

- [ ] **Step 2: Run tests (no regressions)**

Run: `./run_tests.sh` → still green.

- [ ] **Step 3: Visual verification deferred**

The overlay is exercised in Task 16 (HUD F1 toggle) where a round is running. Record in the commit message that visual verification happens there.

- [ ] **Step 4: Commit**

```bash
git add scripts/robot/sensors/debug_overlay.gd scenes/robots/robot.tscn
git commit -m "feat: sensor debug overlay (visual check in HUD task)

Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>"
```

---

### Task 10: Reactive steering and motion controller

**Files:**
- Create: `scripts/robot/steering.gd`, `scripts/robot/motion.gd`
- Test: `tests/unit/steering_test.gd`, `tests/unit/motion_test.gd`

**Interfaces:**
- Consumes: `Blackboard` fields (`position`, `yaw`, `time`, lidar, ultrasonic, bumper, `goal_pos`, `goal_active`), writes `bb.stuck`, `bb.goal_reached`.
- Produces:
  - `class_name Steering`
    - `static func pick_heading(target_angle: float, ranges: PackedFloat32Array, angles: PackedFloat32Array, max_range: float, avoid_dist: float = 1.5, window: float = deg_to_rad(20.0)) -> float` robot-relative heading to drive toward
    - `static func speed_scale(front_clearance: float, slow_dist: float = 2.0, min_scale: float = 0.2) -> float`
  - `class_name Motion extends RefCounted`
    - `var path_provider: Callable` `(from: Vector3, to: Vector3) -> PackedVector3Array`
    - exports-as-vars: `max_v = 2.0`, `max_w = 3.0`, `k_ang = 2.5`, `waypoint_radius = 0.6`, `arrive_radius = 0.5`, `replan_period = 2.0`, `stuck_window = 4.0`, `stuck_progress = 0.3`, `recovery_time = 0.8`
    - `func update(bb: Blackboard, dt: float) -> Vector2` → (v, ω)
    - `func reset() -> void`
    - `var path: PackedVector3Array` (read-only for overlay/tests)

- [ ] **Step 1: Write failing steering tests**

`tests/unit/steering_test.gd`:

```gdscript
extends GdUnitTestSuite

const N := 36


func _scan(hits: Dictionary) -> Array:
	var ranges := PackedFloat32Array()
	var angles := PackedFloat32Array()
	for i in N:
		ranges.append(hits.get(i, 8.0))
		angles.append(wrapf(TAU * i / N, -PI, PI))
	return [ranges, angles]


func test_no_obstacles_heads_to_target() -> void:
	var s := _scan({})
	assert_float(Steering.pick_heading(0.4, s[0], s[1], 8.0)).is_equal_approx(0.4, 1e-6)


func test_empty_scan_heads_to_target() -> void:
	assert_float(Steering.pick_heading(-1.0, PackedFloat32Array(), PackedFloat32Array(), 8.0)).is_equal_approx(-1.0, 1e-6)


func test_obstacle_ahead_deviates() -> void:
	# obstacle at 1 m spanning -20..+20 deg (indices 0,1,2 and 34,35)
	var s := _scan({0: 1.0, 1: 1.0, 2: 1.0, 34: 1.0, 35: 1.0})
	var h: float = Steering.pick_heading(0.0, s[0], s[1], 8.0)
	assert_float(absf(h)).is_greater_equal(deg_to_rad(30.0))
	assert_float(absf(h)).is_less_equal(deg_to_rad(90.0))


func test_far_obstacle_ignored() -> void:
	var s := _scan({0: 6.0, 1: 6.0, 35: 6.0})
	assert_float(Steering.pick_heading(0.0, s[0], s[1], 8.0)).is_equal_approx(0.0, 1e-6)


func test_prefers_side_with_more_room() -> void:
	# obstacle ahead; right side (negative angles) also cluttered at 1.2 m, left open
	var hits := {0: 1.0, 1: 1.0, 35: 1.0}
	for i in range(27, 35):
		hits[i] = 1.2
	var s := _scan(hits)
	assert_float(Steering.pick_heading(0.0, s[0], s[1], 8.0)).is_greater(0.0)


func test_speed_scale() -> void:
	assert_float(Steering.speed_scale(INF)).is_equal_approx(1.0, 1e-6)
	assert_float(Steering.speed_scale(1.0)).is_equal_approx(0.5, 1e-6)
	assert_float(Steering.speed_scale(0.1)).is_equal_approx(0.2, 1e-6)
```

- [ ] **Step 2: Run, expect failure**

- [ ] **Step 3: Implement steering.gd**

`scripts/robot/steering.gd`:

```gdscript
## Reactive heading selection: score every lidar direction by alignment with
## the target minus a proximity penalty, pick the best. Pure function.
class_name Steering
extends RefCounted


static func pick_heading(target_angle: float, ranges: PackedFloat32Array, angles: PackedFloat32Array,
		max_range: float, avoid_dist: float = 1.5, window: float = deg_to_rad(20.0)) -> float:
	var n: int = ranges.size()
	if n == 0:
		return target_angle
	var best_a: float = target_angle
	var best_s: float = -INF
	for i in n:
		var a: float = angles[i]
		var clearance: float = INF
		for j in n:
			if absf(wrapf(angles[j] - a, -PI, PI)) <= window + 1e-6:
				clearance = minf(clearance, ranges[j])
		if clearance >= max_range - 1e-3:
			clearance = INF
		var penalty: float = 0.0
		if clearance < avoid_dist:
			penalty = 2.0 * (avoid_dist - clearance) / avoid_dist
		var score: float = cos(wrapf(a - target_angle, -PI, PI)) - penalty
		if score > best_s + 1e-9:
			best_s = score
			best_a = a
	# If the target direction itself is clear, use it exactly (avoid quantising to ray angles).
	var target_clear: float = INF
	for j in n:
		if absf(wrapf(angles[j] - target_angle, -PI, PI)) <= window + 1e-6:
			target_clear = minf(target_clear, ranges[j])
	if target_clear >= avoid_dist:
		return target_angle
	return best_a


static func speed_scale(front_clearance: float, slow_dist: float = 2.0, min_scale: float = 0.2) -> float:
	if is_inf(front_clearance):
		return 1.0
	return clampf(front_clearance / slow_dist, min_scale, 1.0)
```

- [ ] **Step 4: Run steering tests, expect pass**

- [ ] **Step 5: Write failing motion tests**

`tests/unit/motion_test.gd`. The test integrates a kinematic unicycle itself so no physics is needed:

```gdscript
extends GdUnitTestSuite


func _straight(from: Vector3, to: Vector3) -> PackedVector3Array:
	return PackedVector3Array([from, to])


func _bb(pos: Vector3, yaw: float, goal: Vector3) -> Blackboard:
	var bb := Blackboard.new()
	bb.position = pos
	bb.yaw = yaw
	bb.goal_pos = goal
	bb.goal_active = true
	return bb


## Integrate (v, w) into the blackboard pose for dt (pure kinematics).
func _step(bb: Blackboard, cmd: Vector2, dt: float) -> void:
	bb.yaw = wrapf(bb.yaw + cmd.y * dt, -PI, PI)
	bb.position += bb.local_to_world_dir(0.0) * cmd.x * dt
	bb.time += dt


func test_reaches_goal_in_open_space() -> void:
	var m := Motion.new()
	m.path_provider = _straight
	var bb := _bb(Vector3.ZERO, 0.0, Vector3(5, 0, -5))
	var reached: bool = false
	for i in 600:  # 10 s
		var cmd: Vector2 = m.update(bb, 1.0 / 60.0)
		if bb.goal_reached:
			reached = true
			break
		_step(bb, cmd, 1.0 / 60.0)
	assert_bool(reached).is_true()
	assert_float(bb.position.distance_to(Vector3(5, 0, -5))).is_less(m.arrive_radius + 0.05)


func test_turns_toward_goal_first() -> void:
	var m := Motion.new()
	m.path_provider = _straight
	var bb := _bb(Vector3.ZERO, 0.0, Vector3(5, 0, 0))  # goal to the right (+X) => negative w
	var cmd: Vector2 = m.update(bb, 1.0 / 60.0)
	assert_float(cmd.y).is_less(0.0)
	assert_float(cmd.x).is_less(m.max_v * 0.5)  # slow while turning


func test_inactive_goal_stops() -> void:
	var m := Motion.new()
	m.path_provider = _straight
	var bb := _bb(Vector3.ZERO, 0.0, Vector3(5, 0, 0))
	bb.goal_active = false
	assert_vector(m.update(bb, 0.016)).is_equal(Vector2.ZERO)


func test_stuck_flag_when_no_progress() -> void:
	var m := Motion.new()
	m.path_provider = _straight
	var bb := _bb(Vector3.ZERO, 0.0, Vector3(0, 0, -5))
	var stuck_seen: bool = false
	for i in 400:  # 6.7 s without moving the pose
		m.update(bb, 1.0 / 60.0)
		bb.time += 1.0 / 60.0
		if bb.stuck:
			stuck_seen = true
			break
	assert_bool(stuck_seen).is_true()
	assert_float(bb.time).is_between(m.stuck_window - 0.1, m.stuck_window + 1.0)


func test_no_stuck_flag_while_progressing() -> void:
	var m := Motion.new()
	m.path_provider = _straight
	var bb := _bb(Vector3.ZERO, 0.0, Vector3(0, 0, -20))
	for i in 300:
		var cmd: Vector2 = m.update(bb, 1.0 / 60.0)
		assert_bool(bb.stuck).is_false()
		_step(bb, cmd, 1.0 / 60.0)


func test_bumper_triggers_reverse_recovery() -> void:
	var m := Motion.new()
	m.path_provider = _straight
	var bb := _bb(Vector3.ZERO, 0.0, Vector3(0, 0, -5))
	bb.bumper_enabled = true
	bb.bumper_hit = true
	bb.bumper_normal = Vector3(0, 0, 1)  # wall ahead pushes us back (+Z)
	var cmd: Vector2 = m.update(bb, 1.0 / 60.0)
	assert_float(cmd.x).is_less(0.0)
	bb.bumper_hit = false
	bb.time += 0.3
	cmd = m.update(bb, 1.0 / 60.0)
	assert_float(cmd.x).is_less(0.0)  # still recovering
	bb.time += 1.0
	cmd = m.update(bb, 1.0 / 60.0)
	assert_float(cmd.x).is_greater(0.0)  # recovery over


func test_ultrasonic_slows_forward_speed() -> void:
	var m := Motion.new()
	m.path_provider = _straight
	var bb := _bb(Vector3.ZERO, 0.0, Vector3(0, 0, -5))
	var fast: Vector2 = m.update(bb, 1.0 / 60.0)
	bb.ultrasonic_enabled = true
	bb.ultrasonic = PackedFloat32Array([0.3, 1.2, 1.2])
	var slow: Vector2 = m.update(bb, 1.0 / 60.0)
	assert_float(slow.x).is_less(fast.x * 0.5)


func test_lidar_obstacle_ahead_causes_turn() -> void:
	var m := Motion.new()
	m.path_provider = _straight
	var bb := _bb(Vector3.ZERO, 0.0, Vector3(0, 0, -5))
	bb.lidar_enabled = true
	bb.lidar_max_range = 8.0
	for i in 36:
		var a: float = wrapf(TAU * i / 36, -PI, PI)
		bb.lidar_angles.append(a)
		bb.lidar_ranges.append(1.0 if absf(a) < deg_to_rad(25) else 8.0)
	var cmd: Vector2 = m.update(bb, 1.0 / 60.0)
	assert_float(absf(cmd.y)).is_greater(0.5)
```

- [ ] **Step 6: Run, expect failure**

- [ ] **Step 7: Implement motion.gd**

`scripts/robot/motion.gd`:

```gdscript
## Goal -> (v, w). Global path from path_provider, local heading from Steering,
## ultrasonic slow-down, bumper recovery, stuck detection.
class_name Motion
extends RefCounted

var path_provider: Callable  # (from: Vector3, to: Vector3) -> PackedVector3Array

var max_v: float = 2.0
var max_w: float = 3.0
var k_ang: float = 2.5
var waypoint_radius: float = 0.6
var arrive_radius: float = 0.5
var replan_period: float = 2.0
var stuck_window: float = 4.0
var stuck_progress: float = 0.3
var recovery_time: float = 0.8
var ultrasonic_slow_dist: float = 0.6

var path: PackedVector3Array = PackedVector3Array()
var _idx: int = 0
var _goal: Vector3 = Vector3.ZERO
var _has_goal: bool = false
var _last_replan: float = -INF
var _progress_dist: float = INF
var _progress_time: float = 0.0
var _recover_until: float = -INF
var _recover_w: float = 0.0


func reset() -> void:
	path = PackedVector3Array()
	_idx = 0
	_has_goal = false
	_progress_dist = INF
	_recover_until = -INF


func _replan(bb: Blackboard) -> void:
	path = path_provider.call(bb.position, _goal) if path_provider.is_valid() else PackedVector3Array([bb.position, _goal])
	_idx = 0
	_last_replan = bb.time


func update(bb: Blackboard, dt: float) -> Vector2:
	bb.goal_reached = false
	if not bb.goal_active:
		bb.stuck = false
		reset()
		return Vector2.ZERO

	# New goal?
	if not _has_goal or _goal.distance_to(bb.goal_pos) > 0.5:
		_goal = bb.goal_pos
		_has_goal = true
		_replan(bb)
		_progress_dist = bb.position.distance_to(_goal)
		_progress_time = bb.time
		bb.stuck = false

	var dist_goal: float = bb.position.distance_to(_goal)
	if dist_goal < arrive_radius:
		bb.goal_reached = true
		bb.stuck = false
		return Vector2.ZERO

	# Stuck detection on progress toward the goal.
	if dist_goal < _progress_dist - stuck_progress:
		_progress_dist = dist_goal
		_progress_time = bb.time
		bb.stuck = false
	elif bb.time - _progress_time > stuck_window:
		bb.stuck = true
		_replan(bb)
		_progress_dist = dist_goal
		_progress_time = bb.time
	else:
		bb.stuck = false

	# Bumper recovery: back off and turn away from the contact.
	if bb.bumper_enabled and bb.bumper_hit and bb.time >= _recover_until:
		_recover_until = bb.time + recovery_time
		var n_angle: float = bb.world_to_local_angle(bb.position + bb.bumper_normal)
		_recover_w = -1.5 if n_angle > 0.0 else 1.5
	if bb.time < _recover_until:
		return Vector2(-0.6, _recover_w)

	# Periodic replan keeps the path fresh if the goal moves slowly.
	if bb.time - _last_replan > replan_period:
		_replan(bb)

	# Waypoint following.
	while _idx < path.size() - 1 and bb.position.distance_to(path[_idx]) < waypoint_radius:
		_idx += 1
	var target: Vector3 = path[_idx] if path.size() > 0 else _goal
	var target_angle: float = bb.world_to_local_angle(target)
	var heading: float = target_angle
	var front: float = INF
	if bb.lidar_enabled and bb.lidar_ranges.size() > 0:
		heading = Steering.pick_heading(target_angle, bb.lidar_ranges, bb.lidar_angles, bb.lidar_max_range)
		front = bb.min_range_in_arc(0.0, deg_to_rad(20.0))
		if front >= bb.lidar_max_range - 1e-3:
			front = INF

	var w: float = clampf(k_ang * heading, -max_w, max_w)
	var align: float = clampf(1.0 - absf(heading) / (PI * 0.5), 0.0, 1.0)
	var v: float = max_v * Steering.speed_scale(front) * align
	if bb.ultrasonic_enabled and bb.ultrasonic.size() > 0 and bb.ultrasonic[0] < ultrasonic_slow_dist:
		v = minf(v, max_v * 0.5 * bb.ultrasonic[0] / ultrasonic_slow_dist)
	return Vector2(v, w)
```

- [ ] **Step 8: Run, expect pass**

Run: `./run_tests.sh`. If `test_reaches_goal_in_open_space` oscillates, lower `k_ang` to 2.0; do not change the test.

- [ ] **Step 9: Commit**

```bash
git add scripts/robot/steering.gd scripts/robot/motion.gd tests/unit/steering_test.gd tests/unit/motion_test.gd
git commit -m "feat: reactive steering and motion controller

Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>"
```

---

### Task 11: FSM base

**Files:**
- Create: `scripts/robot/fsm/fsm_state.gd`, `scripts/robot/fsm/fsm.gd`
- Test: `tests/unit/fsm_test.gd`

**Interfaces:**
- Produces:
  - `class_name FSMState extends RefCounted`: `var name: String`, `func enter(bb: Blackboard) -> void`, `func update(bb: Blackboard, dt: float) -> String` (return `""` to stay, otherwise next state name), `func exit(bb: Blackboard) -> void`.
  - `class_name FSM extends RefCounted`: `func add(state: FSMState) -> void`, `func start(name: String, bb: Blackboard) -> void`, `func update(bb: Blackboard, dt: float) -> void`, `func current_name() -> String`, `var transitions: Array[Dictionary]` `{time, from, to}`, `var transition_counts: Dictionary` `"From->To"` → int, `func force(name: String, bb: Blackboard) -> void`.
  - On every transition, `bb.state_name` is set to the new state name.

- [ ] **Step 1: Write failing tests**

`tests/unit/fsm_test.gd`:

```gdscript
extends GdUnitTestSuite


class Counter extends FSMState:
	var ticks: int = 0
	var entered: int = 0
	var exited: int = 0
	var go_to: String = ""
	var after: int = 0

	func _init(n: String, to: String = "", after_ticks: int = 0) -> void:
		name = n
		go_to = to
		after = after_ticks

	func enter(_bb: Blackboard) -> void:
		entered += 1
		ticks = 0

	func update(_bb: Blackboard, _dt: float) -> String:
		ticks += 1
		if go_to != "" and ticks >= after:
			return go_to
		return ""

	func exit(_bb: Blackboard) -> void:
		exited += 1


func test_start_enters_initial_state() -> void:
	var fsm := FSM.new()
	var a := Counter.new("A")
	fsm.add(a)
	var bb := Blackboard.new()
	fsm.start("A", bb)
	assert_str(fsm.current_name()).is_equal("A")
	assert_int(a.entered).is_equal(1)
	assert_str(bb.state_name).is_equal("A")


func test_transition_calls_exit_enter_and_logs() -> void:
	var fsm := FSM.new()
	var a := Counter.new("A", "B", 3)
	var b := Counter.new("B")
	fsm.add(a)
	fsm.add(b)
	var bb := Blackboard.new()
	fsm.start("A", bb)
	for i in 3:
		bb.time = i * 0.1
		fsm.update(bb, 0.1)
	assert_str(fsm.current_name()).is_equal("B")
	assert_int(a.exited).is_equal(1)
	assert_int(b.entered).is_equal(1)
	assert_int(fsm.transitions.size()).is_equal(1)
	assert_str(fsm.transitions[0]["from"]).is_equal("A")
	assert_str(fsm.transitions[0]["to"]).is_equal("B")
	assert_float(fsm.transitions[0]["time"]).is_equal_approx(0.2, 1e-6)
	assert_int(fsm.transition_counts["A->B"]).is_equal(1)


func test_returning_own_name_does_not_transition() -> void:
	var fsm := FSM.new()
	var a := Counter.new("A", "A", 1)
	fsm.add(a)
	var bb := Blackboard.new()
	fsm.start("A", bb)
	fsm.update(bb, 0.1)
	fsm.update(bb, 0.1)
	assert_int(a.entered).is_equal(1)
	assert_array(fsm.transitions).is_empty()


func test_force_transition() -> void:
	var fsm := FSM.new()
	fsm.add(Counter.new("A"))
	fsm.add(Counter.new("B"))
	var bb := Blackboard.new()
	fsm.start("A", bb)
	fsm.force("B", bb)
	assert_str(fsm.current_name()).is_equal("B")
	assert_int(fsm.transitions.size()).is_equal(1)
```

- [ ] **Step 2: Run, expect failure**

- [ ] **Step 3: Implement**

`scripts/robot/fsm/fsm_state.gd`:

```gdscript
## One state of a finite state machine. Reads/writes only the Blackboard.
class_name FSMState
extends RefCounted

var name: String = ""


func enter(_bb: Blackboard) -> void:
	pass


## Return "" to stay, or the name of the next state.
func update(_bb: Blackboard, _dt: float) -> String:
	return ""


func exit(_bb: Blackboard) -> void:
	pass
```

`scripts/robot/fsm/fsm.gd`:

```gdscript
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
	var next: String = current.update(bb, dt)
	if next != "" and next != current.name:
		_transition(next, bb)


func force(name: String, bb: Blackboard) -> void:
	if current == null or name == current.name:
		return
	_transition(name, bb)


func _transition(to: String, bb: Blackboard) -> void:
	assert(states.has(to), "unknown state %s" % to)
	var from: String = current.name
	current.exit(bb)
	transitions.append({"time": bb.time, "from": from, "to": to})
	var key: String = "%s->%s" % [from, to]
	transition_counts[key] = int(transition_counts.get(key, 0)) + 1
	current = states[to]
	bb.state_name = to
	current.enter(bb)
```

- [ ] **Step 4: Run, expect pass**

- [ ] **Step 5: Commit**

```bash
git add scripts/robot/fsm/fsm.gd scripts/robot/fsm/fsm_state.gd tests/unit/fsm_test.gd
git commit -m "feat: FSM base with transition log

Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>"
```

---

### Task 12: Visibility grid

**Files:**
- Create: `scripts/game/visibility_grid.gd`
- Test: `tests/unit/visibility_grid_test.gd`

**Interfaces:**
- Consumes: `Arena.bounds()`, `Arena.wall_rects()`, an injected line-of-sight `Callable (a: Vector3, b: Vector3) -> bool` (true = clear).
- Produces: `class_name VisibilityGrid extends RefCounted`
  - `var cell_size: float = 2.0`, `var centers: Array[Vector3]`, `var openness: PackedFloat32Array` (0..1), `var visible_from_spawn: Array[bool]`, `var last_viewed: PackedFloat32Array`
  - `func build(bounds: Rect2, walls: Array[Rect2], seeker_spawn: Vector3, los: Callable, open_range: float = 4.0, ray_count: int = 16) -> void`
  - `func mark_viewed(pos: Vector3, yaw: float, fov: float, range: float, now: float, los: Callable) -> int` (returns number of cells marked)
  - `func pick_patrol_target(from: Vector3, now: float) -> Vector3`
  - `func pick_hiding_spot(threat: Vector3, min_threat_dist: float) -> Vector3`
  - `func pick_flee_spot(from: Vector3, threat: Vector3) -> Vector3`
  - `func nearest_cell(p: Vector3) -> int`

- [ ] **Step 1: Write failing tests**

`tests/unit/visibility_grid_test.gd`:

```gdscript
extends GdUnitTestSuite

const BOUNDS := Rect2(-10, -10, 20, 20)
# One wall along the Z axis at x in [-0.15, 0.15], z in [-10, 10], splitting the arena.
const WALL := Rect2(-0.15, -10, 0.3, 20)


## LOS blocked iff the segment crosses x = 0.
func _los(a: Vector3, b: Vector3) -> bool:
	return signf(a.x) == signf(b.x) or absf(a.x) < 1e-6 or absf(b.x) < 1e-6


func _grid() -> VisibilityGrid:
	var g := VisibilityGrid.new()
	g.build(BOUNDS, [WALL], Vector3(-7, 0, 0), _los)
	return g


func test_cells_avoid_walls_and_cover_bounds() -> void:
	var g := _grid()
	assert_int(g.centers.size()).is_greater_equal(60)
	for c in g.centers:
		assert_bool(WALL.grow(0.5).has_point(Vector2(c.x, c.z))).is_false()
		assert_bool(BOUNDS.has_point(Vector2(c.x, c.z))).is_true()


func test_visible_from_spawn_respects_wall() -> void:
	var g := _grid()
	for i in g.centers.size():
		var c: Vector3 = g.centers[i]
		assert_bool(g.visible_from_spawn[i]).is_equal(c.x < 0.0)


func test_openness_lower_next_to_wall() -> void:
	var g := _grid()
	var near: int = g.nearest_cell(Vector3(1, 0, 0))
	var far: int = g.nearest_cell(Vector3(7, 0, 0))
	assert_float(g.openness[near]).is_less(g.openness[far])
	assert_float(g.openness[far]).is_equal_approx(1.0, 1e-6)


func test_patrol_prefers_unviewed_cells() -> void:
	var g := _grid()
	var marked: int = g.mark_viewed(Vector3(-7, 0, 0), 0.0, TAU, 100.0, 10.0, _los)
	assert_int(marked).is_greater(0)
	var t: Vector3 = g.pick_patrol_target(Vector3(-7, 0, 0), 10.0)
	assert_float(t.x).is_greater(0.0)  # the unviewed half


func test_mark_viewed_respects_fov() -> void:
	var g := _grid()
	# facing -Z (yaw 0) with 90 deg fov from the far east side: only cells with z < pos.z get marked
	var pos := Vector3(7, 0, 8)
	g.mark_viewed(pos, 0.0, deg_to_rad(90), 100.0, 5.0, _los)
	for i in g.centers.size():
		if g.last_viewed[i] > 0.0:
			assert_float(g.centers[i].z).is_less(pos.z + 0.01)


func test_hiding_spot_is_hidden_and_far_from_threat() -> void:
	var g := _grid()
	var threat := Vector3(-7, 0, 0)
	var s: Vector3 = g.pick_hiding_spot(threat, 6.0)
	assert_bool(g.visible_from_spawn[g.nearest_cell(s)]).is_false()
	assert_float(s.distance_to(threat)).is_greater_equal(6.0)


func test_hiding_spot_relaxes_distance_if_needed() -> void:
	var g := _grid()
	var s: Vector3 = g.pick_hiding_spot(Vector3(0, 0, 0), 1000.0)
	assert_bool(g.centers.has(s)).is_true()


func test_flee_spot_moves_away_from_threat() -> void:
	var g := _grid()
	var from := Vector3(3, 0, 0)
	var threat := Vector3(1, 0, 0)
	var s: Vector3 = g.pick_flee_spot(from, threat)
	assert_float(s.distance_to(threat)).is_greater(from.distance_to(threat))
```

- [ ] **Step 2: Run, expect failure**

- [ ] **Step 3: Implement**

`scripts/game/visibility_grid.gd`:

```gdscript
## Coarse XZ grid over the arena. Knows, per walkable cell, how open it is
## (fraction of rays reaching open_range against walls), whether the seeker's
## spawn can see it, and when the seeker last viewed it. Pure: LOS is injected.
class_name VisibilityGrid
extends RefCounted

var cell_size: float = 2.0
var centers: Array[Vector3] = []
var openness: PackedFloat32Array = PackedFloat32Array()
var visible_from_spawn: Array[bool] = []
var last_viewed: PackedFloat32Array = PackedFloat32Array()

const NEVER: float = -1000.0


func build(bounds: Rect2, walls: Array[Rect2], seeker_spawn: Vector3, los: Callable,
		open_range: float = 4.0, ray_count: int = 16) -> void:
	centers.clear()
	openness = PackedFloat32Array()
	visible_from_spawn.clear()
	last_viewed = PackedFloat32Array()
	var inner: Rect2 = bounds.grow(-cell_size * 0.5)
	var nx: int = int(floor(bounds.size.x / cell_size))
	var nz: int = int(floor(bounds.size.y / cell_size))
	for ix in nx:
		for iz in nz:
			var p := Vector2(bounds.position.x + (ix + 0.5) * cell_size, bounds.position.y + (iz + 0.5) * cell_size)
			if not inner.has_point(p):
				continue
			var blocked: bool = false
			for w in walls:
				if w.grow(0.5).has_point(p):
					blocked = true
					break
			if blocked:
				continue
			var c := Vector3(p.x, 0.0, p.y)
			centers.append(c)
			var open: int = 0
			for k in ray_count:
				var a: float = TAU * k / ray_count
				var to: Vector3 = c + Vector3(-sin(a), 0.0, -cos(a)) * open_range
				if los.call(c, to):
					open += 1
			openness.append(float(open) / ray_count)
			visible_from_spawn.append(bool(los.call(seeker_spawn, c)))
			last_viewed.append(NEVER)


func nearest_cell(p: Vector3) -> int:
	var best: int = -1
	var best_d: float = INF
	for i in centers.size():
		var d: float = centers[i].distance_squared_to(p)
		if d < best_d:
			best_d = d
			best = i
	return best


func mark_viewed(pos: Vector3, yaw: float, fov: float, range: float, now: float, los: Callable) -> int:
	var marked: int = 0
	var half: float = fov * 0.5
	for i in centers.size():
		var d: Vector3 = centers[i] - pos
		if d.length() > range:
			continue
		var ang: float = wrapf(atan2(-d.x, -d.z) - yaw, -PI, PI)
		if absf(ang) > half:
			continue
		if los.call(pos, centers[i]):
			last_viewed[i] = now
			marked += 1
	return marked


func pick_patrol_target(from: Vector3, now: float) -> Vector3:
	var best: int = 0
	var best_s: float = -INF
	for i in centers.size():
		var stale: float = minf(now - last_viewed[i], 60.0)
		var s: float = stale - 0.5 * from.distance_to(centers[i])
		if s > best_s:
			best_s = s
			best = i
	return centers[best]


func pick_hiding_spot(threat: Vector3, min_threat_dist: float) -> Vector3:
	var best: int = -1
	var best_s: float = -INF
	var min_d: float = min_threat_dist
	while best == -1 and min_d >= 0.0:
		for i in centers.size():
			var dt: float = centers[i].distance_to(threat)
			if dt < min_d:
				continue
			var s: float = -3.0 * openness[i] - (2.0 if visible_from_spawn[i] else 0.0) + 0.05 * dt
			if s > best_s:
				best_s = s
				best = i
		min_d -= 2.0  # relax if nothing qualified
	return centers[maxi(best, 0)]


func pick_flee_spot(from: Vector3, threat: Vector3) -> Vector3:
	var best: int = -1
	var best_s: float = -INF
	var d_from: float = from.distance_to(threat)
	for i in centers.size():
		var dt: float = centers[i].distance_to(threat)
		var df: float = centers[i].distance_to(from)
		if df > 10.0 or dt <= d_from:
			continue
		var s: float = dt - 0.5 * df - 2.0 * openness[i]
		if s > best_s:
			best_s = s
			best = i
	if best == -1:  # fall back: farthest cell from threat
		for i in centers.size():
			var dt: float = centers[i].distance_to(threat)
			if dt > best_s:
				best_s = dt
				best = i
	return centers[best]
```

- [ ] **Step 4: Run, expect pass**

- [ ] **Step 5: Commit**

```bash
git add scripts/game/visibility_grid.gd tests/unit/visibility_grid_test.gd
git commit -m "feat: visibility grid for patrol and hiding-spot selection

Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>"
```

---

### Task 13: Seeker states

**Files:**
- Create: `scripts/robot/fsm/seeker_states.gd`
- Modify: `scripts/robot/perception.gd` (add `var seeking_allowed: bool = false`)
- Test: `tests/unit/seeker_states_test.gd`

**Interfaces:**
- Consumes: `FSM`, `FSMState`, `Blackboard`, `VisibilityGrid`.
- Produces: `class_name SeekerStates` (file holds the context class and all seeker state classes as inner classes, plus a factory):
  - `class SeekerStates.Context extends RefCounted`: `grid: VisibilityGrid`, `los: Callable`, `tag_range: float = 1.5`, `lose_timeout: float = 3.0`, `investigate_timeout: float = 8.0`, `catch_delay: float = 0.5`, `fov: float = deg_to_rad(90)`, `vision_range: float = 10.0`, `local_mark_radius: float = 2.5`, `last_investigated_pos: Vector3`, `last_investigated_time: float = -INF`, `investigate_cooldown: float = 10.0`
  - `static func build(ctx: Context) -> FSM` with states named exactly `"Wait"`, `"Patrol"`, `"Investigate"`, `"Chase"`, `"Catch"`; FSM not yet started.
  - Catch request protocol: in `Chase`, when a target is within `tag_range`, set `bb.catch_request` to the hider id (vision) or `-1` (blob-only, meaning "any hider in range") and transition to `Catch`. `Catch.exit` resets `bb.catch_request = 0`.
  - Blob world position helper: `static func blob_world_pos(bb: Blackboard, blob: Dictionary) -> Vector3`.

- [ ] **Step 1: Add `seeking_allowed` to the blackboard**

In `scripts/robot/perception.gd` under `# --- robot state` add:

```gdscript
var seeking_allowed: bool = false  # set true by GameManager when the head start ends
```

- [ ] **Step 2: Write failing tests**

`tests/unit/seeker_states_test.gd`:

```gdscript
extends GdUnitTestSuite

const BOUNDS := Rect2(-10, -10, 20, 20)


func _los_clear(_a: Vector3, _b: Vector3) -> bool:
	return true


func _ctx() -> SeekerStates.Context:
	var ctx := SeekerStates.Context.new()
	ctx.grid = VisibilityGrid.new()
	ctx.grid.build(BOUNDS, [], Vector3(-7, 0, 0), _los_clear)
	ctx.los = _los_clear
	return ctx


func _bb(pos: Vector3 = Vector3.ZERO) -> Blackboard:
	var bb := Blackboard.new()
	bb.role = "seeker"
	bb.position = pos
	bb.vision_enabled = true
	bb.lidar_enabled = true
	bb.lidar_max_range = 8.0
	return bb


func _seen(id: int, pos: Vector3, bb: Blackboard) -> Dictionary:
	return {"id": id, "role": "hider", "position": pos, "bearing": bb.world_to_local_angle(pos),
		"distance": bb.position.distance_to(pos), "time": bb.time}


func _start(ctx: SeekerStates.Context, bb: Blackboard, state: String) -> FSM:
	var fsm := SeekerStates.build(ctx)
	fsm.start(state, bb)
	return fsm


func _tick(fsm: FSM, bb: Blackboard, seconds: float) -> void:
	var steps: int = int(round(seconds * 60.0))
	for i in steps:
		bb.time += 1.0 / 60.0
		fsm.update(bb, 1.0 / 60.0)


func test_wait_to_patrol_when_seeking_allowed() -> void:
	var bb := _bb()
	var fsm := _start(_ctx(), bb, "Wait")
	_tick(fsm, bb, 1.0)
	assert_str(fsm.current_name()).is_equal("Wait")
	assert_bool(bb.goal_active).is_false()
	bb.seeking_allowed = true
	_tick(fsm, bb, 0.1)
	assert_str(fsm.current_name()).is_equal("Patrol")


func test_patrol_sets_goal_and_repicks_when_reached() -> void:
	var bb := _bb()
	var fsm := _start(_ctx(), bb, "Patrol")
	assert_bool(bb.goal_active).is_true()
	var first: Vector3 = bb.goal_pos
	bb.position = first
	bb.goal_reached = true
	_tick(fsm, bb, 0.1)
	assert_bool(bb.goal_pos.distance_to(first) > 1.0).is_true()


func test_patrol_to_chase_on_seen_hider() -> void:
	var bb := _bb()
	var fsm := _start(_ctx(), bb, "Patrol")
	bb.seen_robots = [_seen(42, Vector3(0, 0, -5), bb)]
	_tick(fsm, bb, 0.1)
	assert_str(fsm.current_name()).is_equal("Chase")
	assert_float(bb.goal_pos.distance_to(Vector3(0, 0, -5))).is_less(0.01)


func test_patrol_to_investigate_on_blob() -> void:
	var bb := _bb()
	bb.vision_enabled = false
	var fsm := _start(_ctx(), bb, "Patrol")
	bb.blobs = [{"angle": 0.0, "distance": 4.0, "width": 0.5, "time": bb.time}]
	_tick(fsm, bb, 0.1)
	assert_str(fsm.current_name()).is_equal("Investigate")
	assert_float(bb.goal_pos.distance_to(Vector3(0, 0, -4))).is_less(0.1)


func test_patrol_ignores_blob_near_caught_hider() -> void:
	var bb := _bb()
	bb.caught_ids = [7]
	bb.last_known[7] = {"position": Vector3(0, 0, -4), "time": 0.0, "role": "hider"}
	var fsm := _start(_ctx(), bb, "Patrol")
	bb.blobs = [{"angle": 0.0, "distance": 4.0, "width": 0.5, "time": bb.time}]
	_tick(fsm, bb, 0.1)
	assert_str(fsm.current_name()).is_equal("Patrol")


func test_investigate_to_patrol_on_timeout() -> void:
	var ctx := _ctx()
	var bb := _bb()
	ctx.last_investigated_pos = Vector3(0, 0, -4)
	var fsm := _start(ctx, bb, "Investigate")
	_tick(fsm, bb, ctx.investigate_timeout + 0.2)
	assert_str(fsm.current_name()).is_equal("Patrol")


func test_investigate_to_chase_on_seen() -> void:
	var bb := _bb()
	var fsm := _start(_ctx(), bb, "Investigate")
	bb.seen_robots = [_seen(42, Vector3(2, 0, -3), bb)]
	_tick(fsm, bb, 0.1)
	assert_str(fsm.current_name()).is_equal("Chase")


func test_chase_follows_and_catches_in_range() -> void:
	var ctx := _ctx()
	var bb := _bb()
	var fsm := _start(ctx, bb, "Chase")
	bb.seen_robots = [_seen(42, Vector3(0, 0, -3), bb)]
	_tick(fsm, bb, 0.1)
	assert_str(fsm.current_name()).is_equal("Chase")
	assert_float(bb.goal_pos.distance_to(Vector3(0, 0, -3))).is_less(0.01)
	bb.seen_robots = [_seen(42, Vector3(0, 0, -1.0), bb)]
	_tick(fsm, bb, 0.05)
	assert_str(fsm.current_name()).is_equal("Catch")
	assert_int(bb.catch_request).is_equal(42)


func test_chase_to_investigate_after_lose_timeout() -> void:
	var ctx := _ctx()
	var bb := _bb()
	var fsm := _start(ctx, bb, "Chase")
	bb.seen_robots = [_seen(42, Vector3(0, 0, -3), bb)]
	_tick(fsm, bb, 0.1)
	bb.seen_robots = []
	_tick(fsm, bb, ctx.lose_timeout - 0.5)
	assert_str(fsm.current_name()).is_equal("Chase")
	_tick(fsm, bb, 1.0)
	assert_str(fsm.current_name()).is_equal("Investigate")
	assert_float(bb.goal_pos.distance_to(Vector3(0, 0, -3))).is_less(0.01)


func test_catch_to_patrol_after_delay_and_clears_request() -> void:
	var ctx := _ctx()
	var bb := _bb()
	bb.catch_request = 42
	var fsm := _start(ctx, bb, "Catch")
	assert_bool(bb.goal_active).is_false()
	_tick(fsm, bb, ctx.catch_delay + 0.1)
	assert_str(fsm.current_name()).is_equal("Patrol")
	assert_int(bb.catch_request).is_equal(0)


func test_chase_without_vision_uses_blobs() -> void:
	var ctx := _ctx()
	var bb := _bb()
	bb.vision_enabled = false
	var fsm := _start(ctx, bb, "Chase")
	bb.blobs = [{"angle": 0.0, "distance": 1.0, "width": 0.5, "time": bb.time}]
	_tick(fsm, bb, 0.05)
	assert_str(fsm.current_name()).is_equal("Catch")
	assert_int(bb.catch_request).is_equal(-1)


func test_chase_with_no_target_and_nothing_known_goes_to_patrol() -> void:
	var ctx := _ctx()
	var bb := _bb()
	var fsm := _start(ctx, bb, "Chase")
	_tick(fsm, bb, ctx.lose_timeout + 0.2)
	assert_str(fsm.current_name()).is_equal("Patrol")
```

- [ ] **Step 3: Run, expect failure**

- [ ] **Step 4: Implement seeker_states.gd**

`scripts/robot/fsm/seeker_states.gd`:

```gdscript
## Seeker decision logic: Wait -> Patrol <-> Investigate / Chase -> Catch -> Patrol.
## All states read only the Blackboard and the shared Context.
class_name SeekerStates
extends RefCounted


class Context extends RefCounted:
	var grid: VisibilityGrid
	var los: Callable
	var tag_range: float = 1.5
	var lose_timeout: float = 3.0
	var investigate_timeout: float = 8.0
	var catch_delay: float = 0.5
	var fov: float = deg_to_rad(90.0)
	var vision_range: float = 10.0
	var local_mark_radius: float = 2.5
	var last_investigated_pos: Vector3 = Vector3.ZERO
	var last_investigated_time: float = -INF
	var investigate_cooldown: float = 10.0
	# chase memory
	var chase_target_id: int = 0
	var chase_last_pos: Vector3 = Vector3.ZERO
	var chase_last_seen: float = -INF


static func blob_world_pos(bb: Blackboard, blob: Dictionary) -> Vector3:
	return bb.position + bb.local_to_world_dir(blob["angle"]) * blob["distance"]


## A blob worth investigating: not where a caught hider sits, not recently investigated.
static func interesting_blob(bb: Blackboard, ctx: Context) -> Dictionary:
	var best: Dictionary = {}
	var best_d: float = INF
	for blob in bb.blobs:
		var p: Vector3 = blob_world_pos(bb, blob)
		var skip: bool = false
		for id in bb.caught_ids:
			if bb.last_known.has(id) and bb.last_known[id]["position"].distance_to(p) < 1.5:
				skip = true
				break
		if skip:
			continue
		if bb.time - ctx.last_investigated_time < ctx.investigate_cooldown and p.distance_to(ctx.last_investigated_pos) < 2.0:
			continue
		if blob["distance"] < best_d:
			best_d = blob["distance"]
			best = blob
	return best


static func nearest_seen_hider(bb: Blackboard) -> Dictionary:
	var best: Dictionary = {}
	for s in bb.seen_hiders():
		if best.is_empty() or s["distance"] < best["distance"]:
			best = s
	return best


static func mark_viewed(bb: Blackboard, ctx: Context) -> void:
	if ctx.grid == null:
		return
	if bb.vision_enabled:
		ctx.grid.mark_viewed(bb.position, bb.yaw, ctx.fov, ctx.vision_range, bb.time, ctx.los)
	ctx.grid.mark_viewed(bb.position, bb.yaw, TAU, ctx.local_mark_radius, bb.time, ctx.los)


class Wait extends FSMState:
	func _init() -> void:
		name = "Wait"

	func enter(bb: Blackboard) -> void:
		bb.goal_active = false

	func update(bb: Blackboard, _dt: float) -> String:
		return "Patrol" if bb.seeking_allowed else ""


class Patrol extends FSMState:
	var ctx: Context

	func _init(c: Context) -> void:
		name = "Patrol"
		ctx = c

	func _pick(bb: Blackboard) -> void:
		bb.goal_pos = ctx.grid.pick_patrol_target(bb.position, bb.time)
		bb.goal_active = true

	func enter(bb: Blackboard) -> void:
		_pick(bb)

	func update(bb: Blackboard, _dt: float) -> String:
		SeekerStates.mark_viewed(bb, ctx)
		if not SeekerStates.nearest_seen_hider(bb).is_empty():
			return "Chase"
		if not SeekerStates.interesting_blob(bb, ctx).is_empty():
			return "Investigate"
		if bb.goal_reached or bb.stuck:
			_pick(bb)
		return ""


class Investigate extends FSMState:
	var ctx: Context
	var _started: float = 0.0
	var _arrived_at: float = -INF

	func _init(c: Context) -> void:
		name = "Investigate"
		ctx = c

	func enter(bb: Blackboard) -> void:
		_started = bb.time
		_arrived_at = -INF
		var blob: Dictionary = SeekerStates.interesting_blob(bb, ctx)
		var target: Vector3
		if not blob.is_empty():
			target = SeekerStates.blob_world_pos(bb, blob)
		elif ctx.chase_last_seen > -INF and bb.time - ctx.chase_last_seen < 30.0:
			target = ctx.chase_last_pos
		else:
			target = ctx.last_investigated_pos
		ctx.last_investigated_pos = target
		ctx.last_investigated_time = bb.time
		bb.goal_pos = target
		bb.goal_active = true

	func update(bb: Blackboard, _dt: float) -> String:
		SeekerStates.mark_viewed(bb, ctx)
		if not SeekerStates.nearest_seen_hider(bb).is_empty():
			return "Chase"
		if bb.time - _started > ctx.investigate_timeout or bb.stuck:
			return "Patrol"
		if bb.goal_reached:
			if _arrived_at < 0.0:
				_arrived_at = bb.time
			elif bb.time - _arrived_at > 1.5:
				return "Patrol"
		return ""


class Chase extends FSMState:
	var ctx: Context
	var _entered: float = 0.0

	func _init(c: Context) -> void:
		name = "Chase"
		ctx = c

	func enter(bb: Blackboard) -> void:
		_entered = bb.time
		ctx.chase_last_seen = -INF
		bb.goal_active = true

	func update(bb: Blackboard, _dt: float) -> String:
		SeekerStates.mark_viewed(bb, ctx)
		var target: Dictionary = SeekerStates.nearest_seen_hider(bb)
		if not target.is_empty():
			ctx.chase_target_id = target["id"]
			ctx.chase_last_pos = target["position"]
			ctx.chase_last_seen = bb.time
			bb.goal_pos = target["position"]
			bb.goal_active = true
			if target["distance"] <= ctx.tag_range:
				bb.catch_request = target["id"]
				return "Catch"
			return ""
		if not bb.vision_enabled:
			var blob: Dictionary = SeekerStates.interesting_blob(bb, ctx)
			if not blob.is_empty():
				ctx.chase_last_pos = SeekerStates.blob_world_pos(bb, blob)
				ctx.chase_last_seen = bb.time
				bb.goal_pos = ctx.chase_last_pos
				bb.goal_active = true
				if blob["distance"] <= ctx.tag_range:
					bb.catch_request = -1
					return "Catch"
				return ""
		# Target lost (or never acquired in this Chase).
		if ctx.chase_last_seen == -INF:
			return "Patrol" if bb.time - _entered > ctx.lose_timeout else ""
		if bb.time - ctx.chase_last_seen > ctx.lose_timeout:
			return "Investigate"
		return ""


class Catch extends FSMState:
	var ctx: Context
	var _entered: float = 0.0

	func _init(c: Context) -> void:
		name = "Catch"
		ctx = c

	func enter(bb: Blackboard) -> void:
		_entered = bb.time
		bb.goal_active = false

	func update(bb: Blackboard, _dt: float) -> String:
		return "Patrol" if bb.time - _entered >= ctx.catch_delay else ""

	func exit(bb: Blackboard) -> void:
		bb.catch_request = 0


static func build(ctx: Context) -> FSM:
	var fsm := FSM.new()
	fsm.add(Wait.new())
	fsm.add(Patrol.new(ctx))
	fsm.add(Investigate.new(ctx))
	fsm.add(Chase.new(ctx))
	fsm.add(Catch.new(ctx))
	return fsm
```

- [ ] **Step 5: Run, expect pass**

Run: `./run_tests.sh`.

- [ ] **Step 6: Commit**

```bash
git add scripts/robot/fsm/seeker_states.gd scripts/robot/perception.gd tests/unit/seeker_states_test.gd
git commit -m "feat: seeker FSM states

Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>"
```

---

### Task 14: Hider strategies and states

**Files:**
- Create: `scripts/robot/fsm/hider_strategy.gd`, `scripts/robot/fsm/hider_states.gd`
- Test: `tests/unit/hider_states_test.gd`

**Interfaces:**
- Produces:
  - `class_name HiderStrategy extends RefCounted`: `var strategy_name: String`, `func pick_spot(bb: Blackboard, ctx: HiderStates.Context) -> Vector3`, `func wants_relocate(bb: Blackboard, ctx: HiderStates.Context) -> bool`, `static func create(name: String) -> HiderStrategy` (`"static"` → `StaticHider`, `"dynamic"` → `DynamicHider`).
  - Inner classes `HiderStrategy.StaticHider`, `HiderStrategy.DynamicHider` (`relocate_period: float = 20.0`).
  - `class_name HiderStates`: `class Context`: `grid`, `los`, `seeker_spawn: Vector3`, `strategy: HiderStrategy`, `flee_clear_timeout: float = 4.0`, `threat_blob_dist: float = 4.0`, `min_hide_dist: float = 8.0`, `last_relocate_time: float = 0.0`, `flee_refresh: float = 1.0`.
  - `static func build(ctx: Context) -> FSM` with states `"PickSpot"`, `"GoToSpot"`, `"Hide"`, `"Flee"`, `"Relocate"`.

- [ ] **Step 1: Write failing tests**

`tests/unit/hider_states_test.gd`:

```gdscript
extends GdUnitTestSuite

const BOUNDS := Rect2(-10, -10, 20, 20)
const WALL := Rect2(-0.15, -10, 0.3, 20)


func _los(a: Vector3, b: Vector3) -> bool:
	return signf(a.x) == signf(b.x)


func _ctx(strategy: String = "static") -> HiderStates.Context:
	var ctx := HiderStates.Context.new()
	ctx.grid = VisibilityGrid.new()
	ctx.seeker_spawn = Vector3(-7, 0, 0)
	ctx.grid.build(BOUNDS, [WALL], ctx.seeker_spawn, _los)
	ctx.los = _los
	ctx.strategy = HiderStrategy.create(strategy)
	return ctx


func _bb(pos: Vector3 = Vector3(5, 0, 5)) -> Blackboard:
	var bb := Blackboard.new()
	bb.role = "hider"
	bb.position = pos
	bb.vision_enabled = true
	bb.lidar_enabled = true
	return bb


func _seeker_seen(bb: Blackboard, pos: Vector3) -> void:
	bb.seen_robots = [{"id": 1, "role": "seeker", "position": pos, "bearing": bb.world_to_local_angle(pos),
		"distance": bb.position.distance_to(pos), "time": bb.time}]
	bb.seeker_last_seen_pos = pos
	bb.seeker_last_seen_time = bb.time


func _start(ctx: HiderStates.Context, bb: Blackboard, state: String) -> FSM:
	var fsm := HiderStates.build(ctx)
	fsm.start(state, bb)
	return fsm


func _tick(fsm: FSM, bb: Blackboard, seconds: float) -> void:
	for i in int(round(seconds * 60.0)):
		bb.time += 1.0 / 60.0
		fsm.update(bb, 1.0 / 60.0)


func test_pickspot_sets_hidden_goal_and_goes_to_gotospot() -> void:
	var ctx := _ctx()
	var bb := _bb()
	var fsm := _start(ctx, bb, "PickSpot")
	_tick(fsm, bb, 0.05)
	assert_str(fsm.current_name()).is_equal("GoToSpot")
	assert_bool(bb.goal_active).is_true()
	assert_float(bb.goal_pos.x).is_greater(0.0)  # hidden side of the wall
	assert_float(bb.goal_pos.distance_to(ctx.seeker_spawn)).is_greater_equal(ctx.min_hide_dist)


func test_gotospot_to_hide_on_reached() -> void:
	var bb := _bb()
	var fsm := _start(_ctx(), bb, "PickSpot")
	_tick(fsm, bb, 0.05)
	bb.goal_reached = true
	_tick(fsm, bb, 0.05)
	assert_str(fsm.current_name()).is_equal("Hide")
	assert_bool(bb.goal_active).is_false()


func test_gotospot_to_flee_when_seeker_seen() -> void:
	var bb := _bb()
	var fsm := _start(_ctx(), bb, "PickSpot")
	_tick(fsm, bb, 0.05)
	_seeker_seen(bb, Vector3(3, 0, 5))
	_tick(fsm, bb, 0.05)
	assert_str(fsm.current_name()).is_equal("Flee")


func test_gotospot_to_pickspot_when_stuck() -> void:
	var bb := _bb()
	var fsm := _start(_ctx(), bb, "PickSpot")
	_tick(fsm, bb, 0.05)
	bb.stuck = true
	_tick(fsm, bb, 1.0 / 60.0)  # one tick: GoToSpot -> PickSpot
	assert_str(fsm.current_name()).is_equal("PickSpot")
	bb.stuck = false  # Motion clears the flag once a new goal arrives
	_tick(fsm, bb, 1.0 / 60.0)  # PickSpot -> GoToSpot
	assert_str(fsm.current_name()).is_equal("GoToSpot")
	assert_int(fsm.transition_counts.get("GoToSpot->PickSpot", 0)).is_equal(1)


func test_hide_to_flee_when_seeker_seen_and_flee_goal_is_farther() -> void:
	var bb := _bb(Vector3(3, 0, 0))
	var fsm := _start(_ctx(), bb, "Hide")
	var seeker := Vector3(1, 0, 0)
	_seeker_seen(bb, seeker)
	_tick(fsm, bb, 0.05)
	assert_str(fsm.current_name()).is_equal("Flee")
	assert_bool(bb.goal_active).is_true()
	assert_float(bb.goal_pos.distance_to(seeker)).is_greater(bb.position.distance_to(seeker))


func test_hide_to_relocate_on_approaching_blob() -> void:
	var ctx := _ctx()
	var bb := _bb()
	var fsm := _start(ctx, bb, "Hide")
	bb.blobs = [{"angle": 0.0, "distance": ctx.threat_blob_dist - 0.5, "width": 0.5, "time": bb.time}]
	_tick(fsm, bb, 0.05)
	assert_str(fsm.current_name()).is_equal("GoToSpot")
	assert_int(fsm.transition_counts.get("Hide->Relocate", 0)).is_equal(1)
	assert_int(fsm.transition_counts.get("Relocate->GoToSpot", 0)).is_equal(1)


func test_flee_to_pickspot_after_seeker_unseen_timeout() -> void:
	var ctx := _ctx()
	var bb := _bb(Vector3(3, 0, 0))
	_seeker_seen(bb, Vector3(1, 0, 0))
	var fsm := _start(ctx, bb, "Flee")
	bb.seen_robots = []
	_tick(fsm, bb, ctx.flee_clear_timeout - 0.5)
	assert_str(fsm.current_name()).is_equal("Flee")
	_tick(fsm, bb, 1.0)
	assert_str(fsm.current_name()).is_equal("GoToSpot")
	assert_int(fsm.transition_counts.get("Flee->PickSpot", 0)).is_equal(1)


func test_static_strategy_never_relocates_on_timer() -> void:
	var ctx := _ctx("static")
	var bb := _bb()
	var fsm := _start(ctx, bb, "Hide")
	_tick(fsm, bb, 60.0)
	assert_str(fsm.current_name()).is_equal("Hide")


func test_dynamic_strategy_relocates_after_period() -> void:
	var ctx := _ctx("dynamic")
	var bb := _bb()
	var fsm := _start(ctx, bb, "Hide")
	_tick(fsm, bb, 21.0)
	assert_int(fsm.transition_counts.get("Hide->Relocate", 0)).is_greater_equal(1)


func test_dynamic_pick_spot_avoids_current_position() -> void:
	var ctx := _ctx("dynamic")
	var bb := _bb()
	var first: Vector3 = ctx.strategy.pick_spot(bb, ctx)
	bb.position = first
	var second: Vector3 = ctx.strategy.pick_spot(bb, ctx)
	assert_float(second.distance_to(first)).is_greater(1.5)


func test_strategy_factory_names() -> void:
	assert_str(HiderStrategy.create("static").strategy_name).is_equal("static")
	assert_str(HiderStrategy.create("dynamic").strategy_name).is_equal("dynamic")
```

- [ ] **Step 2: Run, expect failure**

- [ ] **Step 3: Implement hider_strategy.gd**

`scripts/robot/fsm/hider_strategy.gd`:

```gdscript
## Swappable hiding-spot policy used by the hider FSM.
class_name HiderStrategy
extends RefCounted

var strategy_name: String = "base"


static func create(kind: String) -> HiderStrategy:
	match kind:
		"dynamic":
			return DynamicHider.new()
		_:
			return StaticHider.new()


func _threat(bb: Blackboard, ctx: HiderStates.Context) -> Vector3:
	return bb.seeker_last_seen_pos if bb.seeker_last_seen_time > -INF else ctx.seeker_spawn


func pick_spot(bb: Blackboard, ctx: HiderStates.Context) -> Vector3:
	return ctx.grid.pick_hiding_spot(_threat(bb, ctx), ctx.min_hide_dist)


func wants_relocate(_bb: Blackboard, _ctx: HiderStates.Context) -> bool:
	return false


## Picks the least visible spot far from the seeker's spawn and stays there.
class StaticHider extends HiderStrategy:
	func _init() -> void:
		strategy_name = "static"


## Periodically moves to a new spot far from the seeker's last known position.
class DynamicHider extends HiderStrategy:
	var relocate_period: float = 20.0

	func _init() -> void:
		strategy_name = "dynamic"

	func pick_spot(bb: Blackboard, ctx: HiderStates.Context) -> Vector3:
		var threat: Vector3 = _threat(bb, ctx)
		var spot: Vector3 = ctx.grid.pick_hiding_spot(threat, ctx.min_hide_dist)
		if spot.distance_to(bb.position) < 1.5:
			spot = ctx.grid.pick_flee_spot(bb.position, threat)
		return spot

	func wants_relocate(bb: Blackboard, ctx: HiderStates.Context) -> bool:
		return bb.time - ctx.last_relocate_time >= relocate_period
```

- [ ] **Step 4: Implement hider_states.gd**

`scripts/robot/fsm/hider_states.gd`:

```gdscript
## Hider decision logic: PickSpot -> GoToSpot -> Hide; Hide -> Flee / Relocate.
class_name HiderStates
extends RefCounted


class Context extends RefCounted:
	var grid: VisibilityGrid
	var los: Callable
	var seeker_spawn: Vector3 = Vector3.ZERO
	var strategy: HiderStrategy
	var flee_clear_timeout: float = 4.0
	var threat_blob_dist: float = 4.0
	var min_hide_dist: float = 8.0
	var last_relocate_time: float = 0.0
	var flee_refresh: float = 1.0


static func threat_blob(bb: Blackboard, ctx: Context) -> bool:
	for blob in bb.blobs:
		if blob["distance"] < ctx.threat_blob_dist:
			return true
	return false


class PickSpot extends FSMState:
	var ctx: Context

	func _init(c: Context) -> void:
		name = "PickSpot"
		ctx = c

	func enter(bb: Blackboard) -> void:
		bb.goal_pos = ctx.strategy.pick_spot(bb, ctx)
		bb.goal_active = true

	func update(_bb: Blackboard, _dt: float) -> String:
		return "GoToSpot"


class GoToSpot extends FSMState:
	func _init() -> void:
		name = "GoToSpot"

	func update(bb: Blackboard, _dt: float) -> String:
		if not bb.seen_seeker().is_empty():
			return "Flee"
		if bb.goal_reached:
			return "Hide"
		if bb.stuck:
			return "PickSpot"
		return ""


class Hide extends FSMState:
	var ctx: Context

	func _init(c: Context) -> void:
		name = "Hide"
		ctx = c

	func enter(bb: Blackboard) -> void:
		bb.goal_active = false

	func update(bb: Blackboard, _dt: float) -> String:
		if not bb.seen_seeker().is_empty():
			return "Flee"
		if HiderStates.threat_blob(bb, ctx):
			return "Relocate"
		if ctx.strategy.wants_relocate(bb, ctx):
			return "Relocate"
		return ""


class Flee extends FSMState:
	var ctx: Context
	var _last_refresh: float = -INF

	func _init(c: Context) -> void:
		name = "Flee"
		ctx = c

	func _refresh(bb: Blackboard) -> void:
		var threat: Vector3 = bb.seeker_last_seen_pos
		bb.goal_pos = ctx.grid.pick_flee_spot(bb.position, threat)
		bb.goal_active = true
		_last_refresh = bb.time

	func enter(bb: Blackboard) -> void:
		_refresh(bb)

	func update(bb: Blackboard, _dt: float) -> String:
		if bb.time_since_seeker_seen() > ctx.flee_clear_timeout:
			return "PickSpot"
		if (not bb.seen_seeker().is_empty() and bb.time - _last_refresh > ctx.flee_refresh) or bb.goal_reached or bb.stuck:
			_refresh(bb)
		return ""


class Relocate extends FSMState:
	var ctx: Context

	func _init(c: Context) -> void:
		name = "Relocate"
		ctx = c

	func enter(bb: Blackboard) -> void:
		ctx.last_relocate_time = bb.time
		bb.goal_pos = ctx.strategy.pick_spot(bb, ctx)
		bb.goal_active = true

	func update(_bb: Blackboard, _dt: float) -> String:
		return "GoToSpot"


static func build(ctx: Context) -> FSM:
	var fsm := FSM.new()
	fsm.add(PickSpot.new(ctx))
	fsm.add(GoToSpot.new())
	fsm.add(Hide.new(ctx))
	fsm.add(Flee.new(ctx))
	fsm.add(Relocate.new(ctx))
	return fsm
```

Note on `test_dynamic_strategy_relocates_after_period`: `Hide.enter` does not reset `last_relocate_time`, so with the default `0.0` the first relocation happens at t ≈ 20 s. `Relocate.enter` resets it.

- [ ] **Step 5: Run, expect pass**

- [ ] **Step 6: Commit**

```bash
git add scripts/robot/fsm/hider_strategy.gd scripts/robot/fsm/hider_states.gd tests/unit/hider_states_test.gd
git commit -m "feat: hider strategies and FSM states

Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>"
```

---

### Task 15: Round config and robot brain wiring

**Files:**
- Create: `scripts/game/round_config.gd`, `scripts/robot/robot.gd`, `scripts/robot/fsm/goto_state.gd`, `tests/fixtures/nav_arena_helper.gd`
- Modify: `scenes/robots/robot.tscn` (attach `robot.gd` to `Brain`)
- Test: `tests/unit/round_config_test.gd`, `tests/scene/robot_brain_test.gd`

**Interfaces:**
- Produces:
  - `class_name RoundConfig extends Resource` with exports: `variant_name: String = "default"`, `seed: int = 0`, `hider_count: int = 2`, `head_start: float = 15.0`, `round_time: float = 120.0`, `tag_range: float = 1.5`, `prop_count: int = 12`, `seeker_sensors: PackedStringArray = ["lidar", "vision", "ultrasonic", "bumper"]`, `hider_sensors: PackedStringArray = ["lidar", "vision", "ultrasonic", "bumper"]`, `hider_strategy: String = "static"`, `noise_std: float = 0.0`; `func has_sensor(role: String, sensor: String) -> bool`; `func sensors_label(role: String) -> String` (sensor names joined with `+`, e.g. `lidar+vision+ultrasonic+bumper`).
  - `class_name GoToState extends FSMState` (name `"GoTo"`): holds `target: Vector3`; `enter` sets the goal; never transitions. Used by tests and debugging.
  - `class_name RobotBrain extends Node` (script on `Brain`):
    - `var bb: Blackboard`, `var fsm: FSM`, `var motion: Motion`, `var body: DriveBody`, `var active: bool = false`, `var caught: bool = false`, `var strategy_name: String = ""`, `var stuck_events: int = 0`
    - `func setup(role: String, rng: RandomNumberGenerator, nav: NavService, grid: VisibilityGrid, los: Callable, config: RoundConfig, seeker_spawn: Vector3) -> void` wires sensors (enable flags + noise from config), builds the role FSM (seeker starts in `Wait`, hider in `PickSpot`), sets `motion.path_provider = nav.get_path`.
    - `func setup_goto(nav: NavService, goal: Vector3) -> void` test/debug mode: all sensors on, FSM = single `GoTo` state.
    - `func set_caught() -> void`: stops, greys the chassis, `bb.state_name = "Caught"`.
    - `func stats() -> Dictionary` `{distance, collisions, stuck_events, transitions, transition_counts, state}`.
    - Ticks in `_physics_process` only when `active`: time, pose, sensors, blobs, FSM, motion, drive, overlay.
  - `tests/fixtures/nav_arena_helper.gd` (`class_name NavArenaHelper`): `static func ready_arena(suite: GdUnitTestSuite) -> Array` → `[runner, arena]` with navmesh baked and ready (shared by later scene tests).

- [ ] **Step 1: Write failing round config tests**

`tests/unit/round_config_test.gd`:

```gdscript
extends GdUnitTestSuite


func test_defaults() -> void:
	var c := RoundConfig.new()
	assert_int(c.hider_count).is_equal(2)
	assert_bool(c.has_sensor("seeker", "vision")).is_true()
	assert_bool(c.has_sensor("hider", "lidar")).is_true()
	assert_str(c.sensors_label("seeker")).is_equal("lidar+vision+ultrasonic+bumper")


func test_ablation_removes_sensor() -> void:
	var c := RoundConfig.new()
	c.seeker_sensors = PackedStringArray(["lidar", "ultrasonic", "bumper"])
	assert_bool(c.has_sensor("seeker", "vision")).is_false()
	assert_bool(c.has_sensor("hider", "vision")).is_true()
	assert_str(c.sensors_label("seeker")).is_equal("lidar+ultrasonic+bumper")


func test_duplicate_is_independent() -> void:
	var a := RoundConfig.new()
	var b: RoundConfig = a.duplicate()
	b.seed = 5
	assert_int(a.seed).is_equal(0)
```

- [ ] **Step 2: Run, expect failure**

- [ ] **Step 3: Implement round_config.gd and goto_state.gd**

`scripts/game/round_config.gd`:

```gdscript
## Everything that varies between rounds. Saved as .tres for experiments.
class_name RoundConfig
extends Resource

@export var variant_name: String = "default"
@export var seed: int = 0
@export var hider_count: int = 2
@export var head_start: float = 15.0
@export var round_time: float = 120.0
@export var tag_range: float = 1.5
@export var prop_count: int = 12
@export var seeker_sensors: PackedStringArray = PackedStringArray(["lidar", "vision", "ultrasonic", "bumper"])
@export var hider_sensors: PackedStringArray = PackedStringArray(["lidar", "vision", "ultrasonic", "bumper"])
@export var hider_strategy: String = "static"  # "static" | "dynamic"
@export var noise_std: float = 0.0


func has_sensor(role: String, sensor: String) -> bool:
	var list: PackedStringArray = seeker_sensors if role == "seeker" else hider_sensors
	return list.has(sensor)


func sensors_label(role: String) -> String:
	var list: PackedStringArray = seeker_sensors if role == "seeker" else hider_sensors
	return "+".join(list)
```

`scripts/robot/fsm/goto_state.gd`:

```gdscript
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
```

- [ ] **Step 4: Run config tests, expect pass**

- [ ] **Step 5: Write the fixture helper and failing brain scene tests**

`tests/fixtures/nav_arena_helper.gd`:

```gdscript
## Shared setup for scene tests that need a baked, ready arena.
class_name NavArenaHelper
extends RefCounted

const ARENA := preload("res://scenes/arena/arena.tscn")
const ROBOT := preload("res://scenes/robots/robot.tscn")


static func ready_arena(suite: GdUnitTestSuite) -> Array:
	var runner: GdUnitSceneRunner = suite.scene_runner(ARENA.instantiate())
	var arena: Arena = runner.scene()
	arena.nav.bake(arena.static_root)
	for i in 30:
		await runner.simulate_frames(1)
		if arena.nav.is_ready():
			break
	assert(arena.nav.is_ready(), "navmesh never became ready")
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
```

`tests/scene/robot_brain_test.gd`:

```gdscript
extends GdUnitTestSuite

const CRATE := preload("res://scenes/props/crate.tscn")


func test_goto_reaches_goal_around_a_crate() -> void:
	var r := await NavArenaHelper.ready_arena(self)
	var runner: GdUnitSceneRunner = r[0]
	var arena: Arena = r[1]
	# Room SW, straight line along z = 8 from (-13, 8) to (-5, 8); a 1.5 x 1 m crate at (-9, 8)
	# sits on the line, 2 m south of the z = 5 divider and north of the nook wall (z >= 10).
	var crate: StaticBody3D = CRATE.instantiate()
	crate.scale = Vector3(1.5, 1.0, 1.0)
	crate.position = Vector3(-9, 0.5, 8)
	arena.props_root.add_child(crate)
	var body := NavArenaHelper.spawn_robot(arena, Vector3(-13, 0, 8), "hider")
	body.rotation.y = PI / 2  # face -X initially (away from goal) to exercise turning
	var brain: RobotBrain = body.get_node("Brain")
	await runner.simulate_frames(2)
	brain.setup_goto(arena.nav, Vector3(-5, 0, 8))
	brain.active = true
	var reached: bool = false
	for i in 20:  # up to 20 s
		await runner.simulate_frames(60)
		if brain.bb.goal_reached:
			reached = true
			break
	assert_bool(reached).append_failure_message("final pos %s" % body.global_position).is_true()
	assert_int(body.collision_count).is_less_equal(1)


func test_hider_leaves_spawn_and_hides() -> void:
	var r := await NavArenaHelper.ready_arena(self)
	var runner: GdUnitSceneRunner = r[0]
	var arena: Arena = r[1]
	var grid := VisibilityGrid.new()
	var los := NavArenaHelper.los_walls(arena)
	grid.build(arena.bounds(), arena.wall_rects(), arena.seeker_spawn_point(), los)
	var spawn: Vector3 = arena.hider_spawn_points()[0]
	var body := NavArenaHelper.spawn_robot(arena, spawn, "hider")
	var brain: RobotBrain = body.get_node("Brain")
	var rng := RandomNumberGenerator.new()
	rng.seed = 1
	await runner.simulate_frames(2)
	brain.setup("hider", rng, arena.nav, grid, los, RoundConfig.new(), arena.seeker_spawn_point())
	brain.active = true
	await runner.simulate_frames(60)
	assert_str(brain.fsm.current_name()).is_equal("GoToSpot")
	for i in 30:
		await runner.simulate_frames(60)
		if brain.fsm.current_name() == "Hide":
			break
	assert_str(brain.fsm.current_name()).is_equal("Hide")
	assert_float(body.global_position.distance_to(spawn)).is_greater(2.0)
	assert_bool(grid.visible_from_spawn[grid.nearest_cell(body.global_position)]).is_false()


func test_seeker_waits_until_allowed_then_patrols() -> void:
	var r := await NavArenaHelper.ready_arena(self)
	var runner: GdUnitSceneRunner = r[0]
	var arena: Arena = r[1]
	var grid := VisibilityGrid.new()
	var los := NavArenaHelper.los_walls(arena)
	grid.build(arena.bounds(), arena.wall_rects(), arena.seeker_spawn_point(), los)
	var body := NavArenaHelper.spawn_robot(arena, arena.seeker_spawn_point(), "seeker")
	var brain: RobotBrain = body.get_node("Brain")
	var rng := RandomNumberGenerator.new()
	rng.seed = 2
	await runner.simulate_frames(2)
	brain.setup("seeker", rng, arena.nav, grid, los, RoundConfig.new(), arena.seeker_spawn_point())
	brain.active = true
	var start: Vector3 = body.global_position
	await runner.simulate_frames(120)
	assert_str(brain.fsm.current_name()).is_equal("Wait")
	assert_float(body.global_position.distance_to(start)).is_less(0.1)
	brain.bb.seeking_allowed = true
	await runner.simulate_frames(180)
	assert_str(brain.fsm.current_name()).is_equal("Patrol")
	assert_float(body.global_position.distance_to(start)).is_greater(1.0)
	assert_float(body.distance_travelled).is_greater(1.0)


func test_sensor_ablation_flags_reach_blackboard() -> void:
	var r := await NavArenaHelper.ready_arena(self)
	var runner: GdUnitSceneRunner = r[0]
	var arena: Arena = r[1]
	var cfg := RoundConfig.new()
	cfg.seeker_sensors = PackedStringArray(["lidar", "bumper"])
	var body := NavArenaHelper.spawn_robot(arena, arena.seeker_spawn_point(), "seeker")
	var brain: RobotBrain = body.get_node("Brain")
	await runner.simulate_frames(2)
	brain.setup("seeker", RandomNumberGenerator.new(), arena.nav, VisibilityGrid.new(), NavArenaHelper.los_walls(arena), cfg, arena.seeker_spawn_point())
	brain.active = true
	await runner.simulate_frames(3)
	assert_bool(brain.bb.lidar_enabled).is_true()
	assert_bool(brain.bb.vision_enabled).is_false()
	assert_bool(brain.bb.ultrasonic_enabled).is_false()
	assert_bool(brain.bb.bumper_enabled).is_true()


func test_set_caught_stops_robot() -> void:
	var r := await NavArenaHelper.ready_arena(self)
	var runner: GdUnitSceneRunner = r[0]
	var arena: Arena = r[1]
	var body := NavArenaHelper.spawn_robot(arena, Vector3(-13, 0, 8), "hider")
	var brain: RobotBrain = body.get_node("Brain")
	await runner.simulate_frames(2)
	brain.setup_goto(arena.nav, Vector3(-5, 0, 8))
	brain.active = true
	await runner.simulate_frames(60)
	brain.set_caught()
	await runner.simulate_frames(30)
	var p: Vector3 = body.global_position
	await runner.simulate_frames(30)
	assert_float(body.global_position.distance_to(p)).is_less(0.05)
	assert_str(brain.stats()["state"]).is_equal("Caught")
```

- [ ] **Step 6: Run, expect failure**

- [ ] **Step 7: Implement robot.gd and attach it**

`scripts/robot/robot.gd`:

```gdscript
## Ties the layers together: sensors -> blackboard -> FSM -> motion -> drive.
## Lives on the Brain node; its parent is the DriveBody.
class_name RobotBrain
extends Node

var bb: Blackboard = Blackboard.new()
var fsm: FSM
var motion: Motion = Motion.new()
var body: DriveBody
var active: bool = false
var caught: bool = false
var strategy_name: String = ""
var stuck_events: int = 0

var _lidar: Lidar
var _vision: VisionCone
var _ultrasonic: Ultrasonic
var _bumper: Bumper
var _overlay: DebugOverlay
var _sensors: Array[SensorBase] = []
var _was_stuck: bool = false


func _ready() -> void:
	body = get_parent() as DriveBody
	_lidar = body.get_node("Sensors/Lidar")
	_vision = body.get_node("Sensors/Vision")
	_ultrasonic = body.get_node("Sensors/Ultrasonic")
	_bumper = body.get_node("Sensors/Bumper")
	_overlay = body.get_node_or_null("DebugOverlay")
	_sensors = [_lidar, _vision, _ultrasonic, _bumper]
	for s in _sensors:
		s.body = body


func _enable_sensors(role: String, config: RoundConfig) -> void:
	_lidar.enabled = config.has_sensor(role, "lidar")
	_vision.enabled = config.has_sensor(role, "vision")
	_ultrasonic.enabled = config.has_sensor(role, "ultrasonic")
	_bumper.enabled = config.has_sensor(role, "bumper")
	for s in _sensors:
		s.noise_std = config.noise_std


func setup(role: String, rng: RandomNumberGenerator, nav: NavService, grid: VisibilityGrid,
		los: Callable, config: RoundConfig, seeker_spawn: Vector3) -> void:
	body.role = role
	bb.role = role
	bb.rng = rng
	_enable_sensors(role, config)
	motion.path_provider = nav.get_path
	motion.max_v = body.model.max_linear_speed() * 0.8
	if role == "seeker":
		var ctx := SeekerStates.Context.new()
		ctx.grid = grid
		ctx.los = los
		ctx.tag_range = config.tag_range
		ctx.fov = deg_to_rad(_vision.fov_deg)
		ctx.vision_range = _vision.max_range
		fsm = SeekerStates.build(ctx)
		fsm.start("Wait", bb)
		_set_color(Color(0.9, 0.2, 0.2))
	else:
		var hctx := HiderStates.Context.new()
		hctx.grid = grid
		hctx.los = los
		hctx.seeker_spawn = seeker_spawn
		hctx.strategy = HiderStrategy.create(config.hider_strategy)
		strategy_name = hctx.strategy.strategy_name
		fsm = HiderStates.build(hctx)
		fsm.start("PickSpot", bb)
		_set_color(Color(0.2, 0.6, 0.9))


func setup_goto(nav: NavService, goal: Vector3) -> void:
	bb.role = body.role
	for s in _sensors:
		s.enabled = true
	motion.path_provider = nav.get_path
	motion.max_v = body.model.max_linear_speed() * 0.8
	fsm = FSM.new()
	fsm.add(GoToState.new(goal))
	fsm.start("GoTo", bb)


func set_caught() -> void:
	caught = true
	bb.goal_active = false
	bb.state_name = "Caught"
	body.set_command(0.0, 0.0)
	_set_color(Color(0.4, 0.4, 0.4))


func _set_color(c: Color) -> void:
	var mesh := body.get_node("Chassis") as MeshInstance3D
	var mat := StandardMaterial3D.new()
	mat.albedo_color = c
	mesh.set_surface_override_material(0, mat)


func stats() -> Dictionary:
	return {
		"distance": body.distance_travelled,
		"collisions": body.collision_count,
		"stuck_events": stuck_events,
		"transitions": fsm.transitions.duplicate() if fsm else [],
		"transition_counts": fsm.transition_counts.duplicate() if fsm else {},
		"state": bb.state_name,
	}


func _physics_process(dt: float) -> void:
	if not active or fsm == null:
		return
	bb.time += dt
	bb.position = body.global_position
	bb.yaw = body.yaw()
	for s in _sensors:
		s.sense(bb)
	bb.blobs = LidarBlobs.detect(bb.lidar_ranges, bb.lidar_angles, bb.lidar_max_range) if bb.lidar_enabled else []
	if caught:
		body.set_command(0.0, 0.0)
		return
	fsm.update(bb, dt)
	var cmd: Vector2 = motion.update(bb, dt)
	body.set_command(cmd.x, cmd.y)
	if bb.stuck and not _was_stuck:
		stuck_events += 1
	_was_stuck = bb.stuck
	if _overlay:
		_overlay.draw(bb, _lidar, _vision)
```

In `robot.tscn`, attach the script to `Brain`:

```
[ext_resource type="Script" path="res://scripts/robot/robot.gd" id="brain"]

[node name="Brain" type="Node" parent="."]
script = ExtResource("brain")
```

- [ ] **Step 8: Run, expect pass**

Run: `./run_tests.sh`. The two navigation-driven tests are the first real integration of motion + sensors + physics. If `test_goto_reaches_goal_around_a_crate` fails by getting stuck on the crate, tune in this order and re-run: `Motion.k_ang` down to 2.0, `Steering.avoid_dist` up to 2.0, `Lidar.ray_count` up to 48. Record the final values in the commit message. Do not loosen the test.

- [ ] **Step 9: Visual check via computer-use-vnc**

Create a scratch scene in the scratchpad (not committed) or temporarily run the `robot_brain_test` scene manually: instantiate the arena, one robot with `setup_goto`, press F6. Watch the robot steer around the crate. Screenshot to `docs/screenshots/goto_around_crate.png`.

- [ ] **Step 10: Commit**

```bash
git add scripts/game/round_config.gd scripts/robot/robot.gd scripts/robot/fsm/goto_state.gd scenes/robots/robot.tscn tests/ docs/screenshots
git commit -m "feat: round config and robot brain wiring

Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>"
```

---

### Task 16: Catch rule and game manager

**Files:**
- Create: `scripts/game/catch_rule.gd`, `scripts/game/game_manager.gd`
- Test: `tests/unit/catch_rule_test.gd`, `tests/scene/game_manager_test.gd`

**Interfaces:**
- Consumes: `Arena`, `NavService`, `PropLayout`, `PropSpawner`, `VisibilityGrid`, `RobotBrain`, `RoundConfig`.
- Produces:
  - `class_name CatchRule`: `static func is_catch(distance: float, tag_range: float, seeker_state: String, los_clear: bool) -> bool`
  - `class_name GameManager extends Node`
    - `signal round_started(config: RoundConfig)`, `signal round_finished(results: Dictionary)`
    - `@export var arena_path: NodePath`
    - `var phase: String` one of `"idle" | "setup" | "head_start" | "seeking" | "finished"`
    - `var time: float` seconds since the round began (includes head start); `func seek_time() -> float` = `max(0, time - head_start)`
    - `var seeker: RobotBrain`, `var hiders: Array[RobotBrain]`, `var grid: VisibilityGrid`, `var config: RoundConfig`, `var rng: RandomNumberGenerator`
    - `func start_round(cfg: RoundConfig) -> void`
    - `func results() -> Dictionary` (valid after `round_finished`):
      ```
      {
        "variant": String, "seed": int, "hider_strategy": String,
        "seeker_sensors": String, "hider_sensors": String,
        "hiders_total": int, "hiders_caught": int, "duration": float (seek-phase seconds),
        "seeker": {"distance", "collisions", "stuck_events", "transitions", "transition_counts"},
        "hiders": [ {"index": int, "caught": bool, "catch_time": float (-1 if survived),
                     "distance", "collisions", "stuck_events", "transitions", "transition_counts"} ]
      }
      ```
    - `func los_clear(a: Vector3, b: Vector3, mask: int) -> bool` raycast at 0.3 m height.

- [ ] **Step 1: Write failing catch rule tests**

`tests/unit/catch_rule_test.gd`:

```gdscript
extends GdUnitTestSuite


func test_catch_when_in_range_visible_and_chasing() -> void:
	assert_bool(CatchRule.is_catch(1.0, 1.5, "Chase", true)).is_true()
	assert_bool(CatchRule.is_catch(1.0, 1.5, "Catch", true)).is_true()


func test_catch_rule_blocked_los() -> void:
	assert_bool(CatchRule.is_catch(1.0, 1.5, "Chase", false)).is_false()


func test_catch_requires_chase_state() -> void:
	assert_bool(CatchRule.is_catch(0.5, 1.5, "Patrol", true)).is_false()
	assert_bool(CatchRule.is_catch(0.5, 1.5, "Wait", true)).is_false()
	assert_bool(CatchRule.is_catch(0.5, 1.5, "Investigate", true)).is_false()


func test_out_of_range() -> void:
	assert_bool(CatchRule.is_catch(1.6, 1.5, "Chase", true)).is_false()
```

- [ ] **Step 2: Run, expect failure; implement catch_rule.gd**

```gdscript
## The single predicate deciding whether a hider is caught.
class_name CatchRule
extends RefCounted


static func is_catch(distance: float, tag_range: float, seeker_state: String, los_clear: bool) -> bool:
	if distance > tag_range:
		return false
	if not los_clear:
		return false
	return seeker_state == "Chase" or seeker_state == "Catch"
```

Run: `./run_tests.sh` → catch rule tests pass.

- [ ] **Step 3: Write failing game manager scene tests**

`tests/scene/game_manager_test.gd`:

```gdscript
extends GdUnitTestSuite

const ARENA := preload("res://scenes/arena/arena.tscn")


func _game(suite_runner_out: Array) -> GameManager:
	var root := Node3D.new()
	var arena: Arena = ARENA.instantiate()
	arena.name = "Arena"
	root.add_child(arena)
	var gm := GameManager.new()
	gm.name = "GameManager"
	gm.arena_path = NodePath("../Arena")
	root.add_child(gm)
	var runner := scene_runner(root)
	suite_runner_out.append(runner)
	return gm


func _short_config(seed: int = 3) -> RoundConfig:
	var c := RoundConfig.new()
	c.seed = seed
	c.hider_count = 1
	c.head_start = 1.0
	c.round_time = 4.0
	c.prop_count = 6
	return c


func test_round_lifecycle_emits_started_and_finished() -> void:
	var out: Array = []
	var gm := _game(out)
	var runner: GdUnitSceneRunner = out[0]
	var started := monitor_signals(gm)
	gm.start_round(_short_config())
	for i in 10 * 60:
		await runner.simulate_frames(1)
		if gm.phase == "finished":
			break
	assert_str(gm.phase).is_equal("finished")
	await assert_signal(started).is_emitted("round_started")
	await assert_signal(started).is_emitted("round_finished")
	var res: Dictionary = gm.results()
	assert_int(res["hiders_total"]).is_equal(1)
	assert_int(res["seed"]).is_equal(3)
	assert_int(res["hiders"].size()).is_equal(1)
	assert_dict(res["seeker"]).contains_keys(["distance", "collisions", "stuck_events", "transitions", "transition_counts"])


func test_round_ends_on_timeout() -> void:
	var out: Array = []
	var gm := _game(out)
	var runner: GdUnitSceneRunner = out[0]
	var cfg := _short_config()
	cfg.seeker_sensors = PackedStringArray([])  # blind seeker cannot catch anyone
	gm.start_round(cfg)
	for i in 10 * 60:
		await runner.simulate_frames(1)
		if gm.phase == "finished":
			break
	var res: Dictionary = gm.results()
	assert_int(res["hiders_caught"]).is_equal(0)
	assert_bool(res["hiders"][0]["caught"]).is_false()
	assert_float(res["hiders"][0]["catch_time"]).is_equal(-1.0)
	assert_float(res["duration"]).is_equal_approx(cfg.round_time, 0.1)


func test_seeker_catches_adjacent_visible_hider() -> void:
	var out: Array = []
	var gm := _game(out)
	var runner: GdUnitSceneRunner = out[0]
	var cfg := _short_config()
	cfg.head_start = 0.5
	cfg.round_time = 10.0
	cfg.prop_count = 0
	gm.start_round(cfg)
	# wait for setup to finish
	for i in 60:
		await runner.simulate_frames(1)
		if gm.phase != "setup":
			break
	var hider: RobotBrain = gm.hiders[0]
	hider.active = false  # freeze it
	var sp: Vector3 = gm.seeker.body.global_position
	hider.body.global_position = sp + gm.seeker.body.forward() * 3.0
	hider.body.rotation.y = gm.seeker.body.rotation.y
	for i in 10 * 60:
		await runner.simulate_frames(1)
		if gm.phase == "finished":
			break
	var res: Dictionary = gm.results()
	assert_int(res["hiders_caught"]).is_equal(1)
	assert_float(res["hiders"][0]["catch_time"]).is_between(0.0, 6.0)
	assert_int(res["seeker"]["transition_counts"].get("Chase->Catch", 0)).is_greater_equal(1)


func test_blind_seeker_touching_hider_does_not_catch() -> void:
	var out: Array = []
	var gm := _game(out)
	var runner: GdUnitSceneRunner = out[0]
	var cfg := _short_config()
	cfg.head_start = 0.5
	cfg.round_time = 3.0
	cfg.prop_count = 0
	cfg.seeker_sensors = PackedStringArray(["bumper"])
	gm.start_round(cfg)
	for i in 60:
		await runner.simulate_frames(1)
		if gm.phase != "setup":
			break
	var hider: RobotBrain = gm.hiders[0]
	hider.active = false
	hider.body.global_position = gm.seeker.body.global_position + gm.seeker.body.forward() * 1.0
	for i in 10 * 60:
		await runner.simulate_frames(1)
		if gm.phase == "finished":
			break
	assert_int(gm.results()["hiders_caught"]).is_equal(0)


func test_spawns_configured_hider_count_and_props() -> void:
	var out: Array = []
	var gm := _game(out)
	var runner: GdUnitSceneRunner = out[0]
	var cfg := _short_config()
	cfg.hider_count = 3
	cfg.prop_count = 10
	gm.start_round(cfg)
	for i in 60:
		await runner.simulate_frames(1)
		if gm.phase != "setup":
			break
	assert_int(gm.hiders.size()).is_equal(3)
	assert_int(gm.arena.props_root.get_child_count()).is_equal(10)
	assert_int(gm.arena.robots_root.get_child_count()).is_equal(4)
```

- [ ] **Step 4: Run, expect failure**

- [ ] **Step 5: Implement game_manager.gd**

`scripts/game/game_manager.gd`:

```gdscript
## Owns the round: RNG, props, navmesh, robots, phases, catch detection, results.
class_name GameManager
extends Node

signal round_started(config: RoundConfig)
signal round_finished(results: Dictionary)

const ROBOT := preload("res://scenes/robots/robot.tscn")

@export var arena_path: NodePath

var arena: Arena
var phase: String = "idle"
var time: float = 0.0
var config: RoundConfig
var rng: RandomNumberGenerator = RandomNumberGenerator.new()
var grid: VisibilityGrid
var seeker: RobotBrain
var hiders: Array[RobotBrain] = []

var _spawner: PropSpawner = PropSpawner.new()
var _catch_times: Array[float] = []
var _results: Dictionary = {}


func _ready() -> void:
	arena = get_node(arena_path) as Arena
	add_child(_spawner)


func seek_time() -> float:
	return maxf(0.0, time - config.head_start) if config else 0.0


func los_clear(a: Vector3, b: Vector3, mask: int) -> bool:
	var q := PhysicsRayQueryParameters3D.create(a + Vector3(0, 0.3, 0), b + Vector3(0, 0.3, 0), mask)
	return arena.get_world_3d().direct_space_state.intersect_ray(q).is_empty()


func _los_walls(a: Vector3, b: Vector3) -> bool:
	return los_clear(a, b, 1)


func start_round(cfg: RoundConfig) -> void:
	config = cfg
	rng = RandomNumberGenerator.new()
	rng.seed = cfg.seed
	time = 0.0
	_catch_times.clear()
	_results = {}
	# Clear previous round.
	for r in hiders:
		r.body.queue_free()
	if seeker:
		seeker.body.queue_free()
	hiders.clear()
	seeker = null
	_spawner.clear(arena.props_root)
	# Props (seeded), keep spawns clear.
	var keep: Array[Vector3] = [arena.seeker_spawn_point()] + arena.hider_spawn_points()
	var layout := PropLayout.generate(rng, cfg.prop_count, arena.bounds(), arena.wall_rects(), keep, PropSpawner.SIZES)
	_spawner.spawn(layout, arena.props_root)
	arena.nav.bake(arena.static_root)
	phase = "setup"


func _spawn_robots() -> void:
	# Seeker.
	var sb: DriveBody = ROBOT.instantiate()
	sb.name = "Seeker"
	sb.role = "seeker"
	sb.position = arena.seeker_spawn_point() + Vector3(0, 0.3, 0)
	arena.robots_root.add_child(sb)
	seeker = sb.get_node("Brain")
	seeker.setup("seeker", rng, arena.nav, grid, _los_walls, config, arena.seeker_spawn_point())
	# Hiders at a seeded random subset of spawn markers.
	var spawns: Array[Vector3] = arena.hider_spawn_points()
	for i in range(spawns.size() - 1, 0, -1):  # Fisher-Yates with our RNG
		var j: int = rng.randi_range(0, i)
		var tmp: Vector3 = spawns[i]
		spawns[i] = spawns[j]
		spawns[j] = tmp
	for i in mini(config.hider_count, spawns.size()):
		var hb: DriveBody = ROBOT.instantiate()
		hb.name = "Hider%d" % i
		hb.role = "hider"
		hb.position = spawns[i] + Vector3(0, 0.3, 0)
		hb.rotation.y = rng.randf_range(-PI, PI)
		arena.robots_root.add_child(hb)
		var brain: RobotBrain = hb.get_node("Brain")
		brain.setup("hider", rng, arena.nav, grid, _los_walls, config, arena.seeker_spawn_point())
		hiders.append(brain)
		_catch_times.append(-1.0)


func _physics_process(dt: float) -> void:
	match phase:
		"setup":
			if not arena.nav.is_ready():
				return
			grid = VisibilityGrid.new()
			grid.build(arena.bounds(), arena.wall_rects(), arena.seeker_spawn_point(), _los_walls)
			_spawn_robots()
			seeker.active = true
			for h in hiders:
				h.active = true
			phase = "head_start"
			round_started.emit(config)
		"head_start":
			time += dt
			if time >= config.head_start:
				phase = "seeking"
				seeker.bb.seeking_allowed = true
		"seeking":
			time += dt
			_check_catches()
			var all_caught: bool = true
			for h in hiders:
				if not h.caught:
					all_caught = false
			if all_caught or seek_time() >= config.round_time:
				_finish()


func _check_catches() -> void:
	var req: int = seeker.bb.catch_request
	if req == 0:
		return
	var sp: Vector3 = seeker.body.global_position
	for i in hiders.size():
		var h: RobotBrain = hiders[i]
		if h.caught:
			continue
		var id: int = h.body.get_instance_id()
		if req != -1 and req != id:
			continue
		var hp: Vector3 = h.body.global_position
		var d: float = Vector2(sp.x, sp.z).distance_to(Vector2(hp.x, hp.z))
		if CatchRule.is_catch(d, config.tag_range, seeker.bb.state_name, los_clear(sp, hp, 3)):
			h.set_caught()
			seeker.bb.caught_ids.append(id)
			_catch_times[i] = seek_time()


func _finish() -> void:
	phase = "finished"
	seeker.active = false
	for h in hiders:
		h.active = false
		h.body.set_command(0.0, 0.0)
	seeker.body.set_command(0.0, 0.0)
	var hider_rows: Array[Dictionary] = []
	var caught: int = 0
	for i in hiders.size():
		var s: Dictionary = hiders[i].stats()
		s["index"] = i
		s["caught"] = hiders[i].caught
		s["catch_time"] = _catch_times[i]
		s.erase("state")
		if hiders[i].caught:
			caught += 1
		hider_rows.append(s)
	var seeker_stats: Dictionary = seeker.stats()
	seeker_stats.erase("state")
	_results = {
		"variant": config.variant_name,
		"seed": config.seed,
		"hider_strategy": config.hider_strategy,
		"seeker_sensors": config.sensors_label("seeker"),
		"hider_sensors": config.sensors_label("hider"),
		"hiders_total": hiders.size(),
		"hiders_caught": caught,
		"duration": seek_time(),
		"seeker": seeker_stats,
		"hiders": hider_rows,
	}
	round_finished.emit(_results)


func results() -> Dictionary:
	return _results
```

Note: `los_clear(sp, hp, 3)` uses mask 1|2 (walls + props), so robots never block the catch ray; the hider itself would otherwise be hit first and read as "blocked".

- [ ] **Step 6: Run, expect pass**

Run: `./run_tests.sh`. The frozen-hider catch test depends on the seeker seeing a robot 3 m ahead at round start: `Wait` → `Patrol` → `Chase` within a second. If it fails, print `gm.seeker.fsm.transitions` in the failure message to see where it stalled before changing anything.

- [ ] **Step 7: Commit**

```bash
git add scripts/game/catch_rule.gd scripts/game/game_manager.gd tests/
git commit -m "feat: game manager round lifecycle and catch rule

Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>"
```

---

### Task 17: HUD, camera and main scene

**Files:**
- Create: `scripts/ui/hud.gd`, `scripts/ui/camera_rig.gd`, `scripts/main.gd`, `scenes/main.tscn`, `experiments/default_round.tres`
- Modify: `project.godot` (set `run/main_scene`, input actions)

**Interfaces:**
- Consumes: `GameManager`, `RobotBrain`, `DebugOverlay.visible_globally`.
- Produces:
  - `scenes/main.tscn`: `Main` (Node3D, `main.gd`) → `Arena` (instance), `GameManager` (`arena_path = "../Arena"`), `CameraRig` (Node3D, `camera_rig.gd`, child `Camera3D`), `HUD` (CanvasLayer, `hud.gd`, child `Label` named `Text`).
  - Input actions in `project.godot`: `toggle_overlay` (F1), `restart_round` (R), `follow_next` (Tab), `free_camera` (0).
  - `main.gd`: loads `res://experiments/default_round.tres`, starts a round on ready, restarts on `restart_round`, toggles overlay, cycles camera follow.
  - `hud.gd`: `func bind(gm: GameManager) -> void`; shows phase, time, seek time, per-robot `name: state`, catches `n/m`.
  - `camera_rig.gd`: orbit with right-mouse drag, zoom with wheel, `func follow(body: Node3D) -> void`, `func free_camera() -> void`.

No automated tests: all UI. Verification is Step 5 via computer-use-vnc.

- [ ] **Step 1: Default round config resource**

`experiments/default_round.tres`:

```
[gd_resource type="Resource" script_class="RoundConfig" load_steps=2 format=3]

[ext_resource type="Script" path="res://scripts/game/round_config.gd" id="cfg"]

[resource]
script = ExtResource("cfg")
variant_name = "default"
seed = 1
hider_count = 3
head_start = 15.0
round_time = 120.0
tag_range = 1.5
prop_count = 12
seeker_sensors = PackedStringArray("lidar", "vision", "ultrasonic", "bumper")
hider_sensors = PackedStringArray("lidar", "vision", "ultrasonic", "bumper")
hider_strategy = "static"
noise_std = 0.0
```

- [ ] **Step 2: Input actions and main scene in project.godot**

Append:

```ini
[input]

toggle_overlay={
"deadzone": 0.5,
"events": [Object(InputEventKey,"resource_local_to_scene":false,"resource_name":"","device":-1,"window_id":0,"alt_pressed":false,"shift_pressed":false,"ctrl_pressed":false,"meta_pressed":false,"pressed":false,"keycode":4194332,"physical_keycode":0,"key_label":0,"unicode":0,"location":0,"echo":false,"script":null)
]
}
restart_round={
"deadzone": 0.5,
"events": [Object(InputEventKey,"resource_local_to_scene":false,"resource_name":"","device":-1,"window_id":0,"alt_pressed":false,"shift_pressed":false,"ctrl_pressed":false,"meta_pressed":false,"pressed":false,"keycode":82,"physical_keycode":0,"key_label":0,"unicode":114,"location":0,"echo":false,"script":null)
]
}
follow_next={
"deadzone": 0.5,
"events": [Object(InputEventKey,"resource_local_to_scene":false,"resource_name":"","device":-1,"window_id":0,"alt_pressed":false,"shift_pressed":false,"ctrl_pressed":false,"meta_pressed":false,"pressed":false,"keycode":4194306,"physical_keycode":0,"key_label":0,"unicode":0,"location":0,"echo":false,"script":null)
]
}
free_camera={
"deadzone": 0.5,
"events": [Object(InputEventKey,"resource_local_to_scene":false,"resource_name":"","device":-1,"window_id":0,"alt_pressed":false,"shift_pressed":false,"ctrl_pressed":false,"meta_pressed":false,"pressed":false,"keycode":48,"physical_keycode":0,"key_label":0,"unicode":48,"location":0,"echo":false,"script":null)
]
}
```

(4194332 = KEY_F1, 82 = KEY_R, 4194306 = KEY_TAB, 48 = KEY_0.) In `[application]` add `run/main_scene="res://scenes/main.tscn"`.

- [ ] **Step 3: Scripts**

`scripts/ui/camera_rig.gd`:

```gdscript
## Orbit camera: right-drag to orbit, wheel to zoom, optional follow target.
class_name CameraRig
extends Node3D

@export var distance: float = 25.0
@export var pitch: float = -55.0
@export var yaw_deg: float = 0.0

var _target: Node3D = null
var _dragging: bool = false
@onready var _cam: Camera3D = $Camera3D


func follow(body: Node3D) -> void:
	_target = body
	distance = minf(distance, 10.0)


func free_camera() -> void:
	_target = null
	distance = 25.0


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
	if _target and is_instance_valid(_target):
		global_position = global_position.lerp(_target.global_position, 0.15)
	elif _target == null:
		global_position = global_position.lerp(Vector3.ZERO, 0.1)
	rotation_degrees = Vector3(0.0, yaw_deg, 0.0)
	# Place the camera `distance` away along a ray tilted by `pitch` below horizontal, looking at the rig origin.
	var pivot := Transform3D.IDENTITY.rotated(Vector3.RIGHT, deg_to_rad(pitch))
	_cam.transform = pivot * Transform3D(Basis.IDENTITY, Vector3(0, 0, distance))
	_cam.look_at(global_position, Vector3.UP)
```

`scripts/ui/hud.gd`:

```gdscript
## Text HUD: phase, timers, per-robot FSM state, catches.
class_name HUD
extends CanvasLayer

var _gm: GameManager
@onready var _text: Label = $Text


func bind(gm: GameManager) -> void:
	_gm = gm


func _process(_dt: float) -> void:
	if _gm == null or _gm.config == null:
		_text.text = "no round"
		return
	var lines: PackedStringArray = []
	lines.append("phase: %s   t=%.1f s   seek=%.1f / %.0f s" % [_gm.phase, _gm.time, _gm.seek_time(), _gm.config.round_time])
	lines.append("seed %d  variant %s  hiders %s" % [_gm.config.seed, _gm.config.variant_name, _gm.config.hider_strategy])
	if _gm.seeker:
		lines.append("Seeker: %s  (sensors %s)" % [_gm.seeker.bb.state_name, _gm.config.sensors_label("seeker")])
	var caught: int = 0
	for i in _gm.hiders.size():
		var h: RobotBrain = _gm.hiders[i]
		if h.caught:
			caught += 1
		lines.append("Hider%d: %s" % [i, h.bb.state_name])
	lines.append("caught %d / %d" % [caught, _gm.hiders.size()])
	lines.append("[F1] overlay  [R] restart  [Tab] follow next  [0] free cam  RMB drag orbit, wheel zoom")
	_text.text = "\n".join(lines)
```

`scripts/main.gd`:

```gdscript
## Visual entry point: start a round from the default config, handle hotkeys.
extends Node3D

const DEFAULT_CONFIG := preload("res://experiments/default_round.tres")

@onready var gm: GameManager = $GameManager
@onready var cam: CameraRig = $CameraRig
@onready var hud: HUD = $HUD

var _follow_index: int = -1


func _ready() -> void:
	hud.bind(gm)
	gm.round_finished.connect(func(res: Dictionary) -> void: print(JSON.stringify(res, "  ")))
	gm.start_round(DEFAULT_CONFIG.duplicate())


func _unhandled_input(event: InputEvent) -> void:
	if event.is_action_pressed("toggle_overlay"):
		DebugOverlay.visible_globally = not DebugOverlay.visible_globally
	elif event.is_action_pressed("restart_round"):
		var cfg: RoundConfig = DEFAULT_CONFIG.duplicate()
		cfg.seed = gm.config.seed + 1 if gm.config else cfg.seed
		gm.start_round(cfg)
	elif event.is_action_pressed("follow_next"):
		var bodies: Array[Node3D] = []
		if gm.seeker:
			bodies.append(gm.seeker.body)
		for h in gm.hiders:
			bodies.append(h.body)
		if bodies.is_empty():
			return
		_follow_index = (_follow_index + 1) % bodies.size()
		cam.follow(bodies[_follow_index])
	elif event.is_action_pressed("free_camera"):
		_follow_index = -1
		cam.free_camera()
```

- [ ] **Step 4: Write main.tscn**

```
[gd_scene load_steps=6 format=3]

[ext_resource type="Script" path="res://scripts/main.gd" id="main"]
[ext_resource type="PackedScene" path="res://scenes/arena/arena.tscn" id="arena"]
[ext_resource type="Script" path="res://scripts/game/game_manager.gd" id="gm"]
[ext_resource type="Script" path="res://scripts/ui/camera_rig.gd" id="cam"]
[ext_resource type="Script" path="res://scripts/ui/hud.gd" id="hud"]

[node name="Main" type="Node3D"]
script = ExtResource("main")

[node name="Arena" parent="." instance=ExtResource("arena")]

[node name="GameManager" type="Node" parent="."]
script = ExtResource("gm")
arena_path = NodePath("../Arena")

[node name="CameraRig" type="Node3D" parent="."]
script = ExtResource("cam")

[node name="Camera3D" type="Camera3D" parent="CameraRig"]
current = true
far = 200.0

[node name="HUD" type="CanvasLayer" parent="."]
script = ExtResource("hud")

[node name="Text" type="Label" parent="HUD"]
offset_left = 12.0
offset_top = 12.0
offset_right = 700.0
offset_bottom = 200.0
theme_override_font_sizes/font_size = 16
```

- [ ] **Step 5: Verify via computer-use-vnc (this is the task's test)**

1. In the editor, Project → Reload Current Project so new input actions and main scene register.
2. Press F5. Expected within 2 s: arena, 12 crates, 1 red seeker frozen at (-7, 0), 3 blue hiders driving away from their spawns. HUD shows `phase: head_start`, hider states `GoToSpot`.
3. At t = 15 s HUD shows `phase: seeking`, seeker `Patrol`, seeker starts moving.
4. Press F1: lidar rays (green/red) and blue vision cones appear on all robots. Screenshot → `docs/screenshots/overlay.png`.
5. Press Tab repeatedly: camera follows seeker, then each hider. Press 0: free camera. Right-drag orbits, wheel zooms.
6. Watch until a catch or timeout: HUD caught counter increments, caught hider turns grey and stops, Output panel prints the results JSON. Screenshot → `docs/screenshots/round.png`.
7. Press R: round restarts with a different prop layout.
8. Note anything that looks wrong (robots stuck on crates, spinning, hiders in the open) in the commit message as known tuning items; do not fix in this task unless it is a crash.

- [ ] **Step 6: Run tests (no regressions) and commit**

```bash
./run_tests.sh
git add scripts/ui scripts/main.gd scenes/main.tscn experiments/default_round.tres project.godot docs/screenshots
git commit -m "feat: HUD, orbit camera and main scene

Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>"
```

---

### Task 18: Results CSV, experiment config and headless batch runner

**Files:**
- Create: `scripts/experiment/results_csv.gd`, `scripts/experiment/experiment_config.gd`, `scripts/experiment/batch_runner.gd`, `scenes/batch.tscn`, `experiments/ablation.tres`, `experiments/strategies.tres`, `experiments/analyze.py`, `run_experiment.sh`
- Test: `tests/unit/results_csv_test.gd`, `tests/scene/determinism_test.gd`

**Interfaces:**
- Produces:
  - `class_name ResultsCsv`: `const MAX_HIDERS := 3`, `static func header() -> PackedStringArray`, `static func row(res: Dictionary) -> PackedStringArray`, `static func transitions_rows(res: Dictionary) -> Array[PackedStringArray]` (one row per FSM transition: variant, seed, robot, time, from, to).
    Columns: `variant, seed, hider_strategy, seeker_sensors, hider_sensors, hiders_total, hiders_caught, duration, seeker_distance, seeker_collisions, seeker_stuck, seeker_transitions, hider0_caught, hider0_catch_time, hider0_distance, hider0_collisions, hider0_stuck, hider0_transitions, hider1_..., hider2_...` (missing hiders → empty strings).
  - `class_name ExperimentConfig extends Resource`: `@export var variants: Array[RoundConfig]`, `@export var rounds_per_variant: int = 20`.
  - `class_name BatchRunner extends Node` (root of `batch.tscn`, children `Arena`, `GameManager`): parses `OS.get_cmdline_user_args()`: `--config <res path>` (required), `--rounds <n>` (overrides `rounds_per_variant`), `--out <path>` (default `experiments/results.csv`), `--time-scale <f>` (default 8), `--log-transitions`, `--seed-offset <n>` (default 0). Runs every variant × seeds `offset..offset+rounds-1`, writes rows as each round ends, prints a summary table, `get_tree().quit(0)`; `quit(2)` on bad args.
  - `run_experiment.sh <config.tres> <rounds> <out.csv> [extra args]` wrapper.

- [ ] **Step 1: Write failing CSV tests**

`tests/unit/results_csv_test.gd`:

```gdscript
extends GdUnitTestSuite


func _res(caught: Array[bool]) -> Dictionary:
	var hiders: Array[Dictionary] = []
	for i in caught.size():
		hiders.append({"index": i, "caught": caught[i], "catch_time": 12.5 if caught[i] else -1.0,
			"distance": 10.0 + i, "collisions": i, "stuck_events": 0,
			"transitions": [{"time": 1.0, "from": "PickSpot", "to": "GoToSpot"}],
			"transition_counts": {"PickSpot->GoToSpot": 1}})
	var n_caught: int = 0
	for c in caught:
		if c:
			n_caught += 1
	return {
		"variant": "all", "seed": 4, "hider_strategy": "static",
		"seeker_sensors": "lidar+vision+ultrasonic+bumper", "hider_sensors": "lidar+vision+ultrasonic+bumper",
		"hiders_total": caught.size(), "hiders_caught": n_caught, "duration": 60.0,
		"seeker": {"distance": 42.0, "collisions": 1, "stuck_events": 2,
			"transitions": [{"time": 15.0, "from": "Wait", "to": "Patrol"}, {"time": 20.0, "from": "Patrol", "to": "Chase"}],
			"transition_counts": {"Wait->Patrol": 1, "Patrol->Chase": 1}},
		"hiders": hiders,
	}


func test_header_and_row_have_same_length() -> void:
	var row := ResultsCsv.row(_res([true, false]))
	assert_int(row.size()).is_equal(ResultsCsv.header().size())


func test_row_values() -> void:
	var h := ResultsCsv.header()
	var row := ResultsCsv.row(_res([true, false]))
	assert_str(row[h.find("variant")]).is_equal("all")
	assert_str(row[h.find("seed")]).is_equal("4")
	assert_str(row[h.find("hiders_caught")]).is_equal("1")
	assert_str(row[h.find("duration")]).is_equal("60.000")
	assert_str(row[h.find("seeker_distance")]).is_equal("42.000")
	assert_str(row[h.find("seeker_transitions")]).is_equal("2")
	assert_str(row[h.find("hider0_caught")]).is_equal("1")
	assert_str(row[h.find("hider0_catch_time")]).is_equal("12.500")
	assert_str(row[h.find("hider1_caught")]).is_equal("0")


func test_csv_row_with_survivors() -> void:
	var h := ResultsCsv.header()
	var row := ResultsCsv.row(_res([false]))
	assert_str(row[h.find("hider0_catch_time")]).is_equal("-1.000")
	assert_str(row[h.find("hider2_caught")]).is_equal("")
	assert_str(row[h.find("hider2_distance")]).is_equal("")


func test_transition_rows() -> void:
	var rows := ResultsCsv.transitions_rows(_res([true]))
	assert_int(rows.size()).is_equal(3)  # 2 seeker + 1 hider
	assert_array(Array(rows[0])).is_equal(["all", "4", "seeker", "15.000", "Wait", "Patrol"])
	assert_array(Array(rows[2])).is_equal(["all", "4", "hider0", "1.000", "PickSpot", "GoToSpot"])
```

- [ ] **Step 2: Run, expect failure; implement results_csv.gd**

```gdscript
## Flattens a GameManager results Dictionary into CSV rows.
class_name ResultsCsv
extends RefCounted

const MAX_HIDERS: int = 3
const HIDER_FIELDS: PackedStringArray = ["caught", "catch_time", "distance", "collisions", "stuck", "transitions"]


static func header() -> PackedStringArray:
	var h: PackedStringArray = ["variant", "seed", "hider_strategy", "seeker_sensors", "hider_sensors",
		"hiders_total", "hiders_caught", "duration",
		"seeker_distance", "seeker_collisions", "seeker_stuck", "seeker_transitions"]
	for i in MAX_HIDERS:
		for f in HIDER_FIELDS:
			h.append("hider%d_%s" % [i, f])
	return h


static func _f(x: float) -> String:
	return "%.3f" % x


static func row(res: Dictionary) -> PackedStringArray:
	var s: Dictionary = res["seeker"]
	var r: PackedStringArray = [
		str(res["variant"]), str(res["seed"]), str(res["hider_strategy"]),
		str(res["seeker_sensors"]), str(res["hider_sensors"]),
		str(res["hiders_total"]), str(res["hiders_caught"]), _f(res["duration"]),
		_f(s["distance"]), str(s["collisions"]), str(s["stuck_events"]), str(s["transitions"].size()),
	]
	var hiders: Array = res["hiders"]
	for i in MAX_HIDERS:
		if i < hiders.size():
			var h: Dictionary = hiders[i]
			r.append("1" if h["caught"] else "0")
			r.append(_f(h["catch_time"]))
			r.append(_f(h["distance"]))
			r.append(str(h["collisions"]))
			r.append(str(h["stuck_events"]))
			r.append(str(h["transitions"].size()))
		else:
			for f in HIDER_FIELDS:
				r.append("")
	return r


static func transitions_header() -> PackedStringArray:
	return PackedStringArray(["variant", "seed", "robot", "time", "from", "to"])


static func transitions_rows(res: Dictionary) -> Array[PackedStringArray]:
	var out: Array[PackedStringArray] = []
	for t in res["seeker"]["transitions"]:
		out.append(PackedStringArray([str(res["variant"]), str(res["seed"]), "seeker", _f(t["time"]), t["from"], t["to"]]))
	var hiders: Array = res["hiders"]
	for i in hiders.size():
		for t in hiders[i]["transitions"]:
			out.append(PackedStringArray([str(res["variant"]), str(res["seed"]), "hider%d" % i, _f(t["time"]), t["from"], t["to"]]))
	return out
```

Run: `./run_tests.sh` → CSV tests pass.

- [ ] **Step 3: Write the failing determinism scene test**

`tests/scene/determinism_test.gd`. Both rounds run in the same scene back to back (as the batch runner does), so leftover bodies from a parallel world cannot perturb the second run:

```gdscript
extends GdUnitTestSuite

const ARENA := preload("res://scenes/arena/arena.tscn")


func _make_game() -> Array:  # [runner, gm]
	var root := Node3D.new()
	var arena: Arena = ARENA.instantiate()
	arena.name = "Arena"
	root.add_child(arena)
	var gm := GameManager.new()
	gm.arena_path = NodePath("../Arena")
	root.add_child(gm)
	var runner := scene_runner(root)
	return [runner, gm]


func _run_round(runner: GdUnitSceneRunner, gm: GameManager, seed: int) -> PackedStringArray:
	var cfg := RoundConfig.new()
	cfg.seed = seed
	cfg.hider_count = 2
	cfg.head_start = 1.0
	cfg.round_time = 6.0
	cfg.prop_count = 8
	gm.start_round(cfg)
	for i in 12 * 60:
		await runner.simulate_frames(1)
		if gm.phase == "finished":
			break
	assert_str(gm.phase).is_equal("finished")
	await runner.simulate_frames(2)  # let queue_free of the previous round settle before the next
	return ResultsCsv.row(gm.results())


func test_same_seed_same_results() -> void:
	var g := _make_game()
	var a := await _run_round(g[0], g[1], 21)
	var b := await _run_round(g[0], g[1], 21)
	assert_array(Array(a)).is_equal(Array(b))


func test_different_seed_different_results() -> void:
	var g := _make_game()
	var a := await _run_round(g[0], g[1], 21)
	var b := await _run_round(g[0], g[1], 22)
	assert_array(Array(a)).is_not_equal(Array(b))
```

- [ ] **Step 4: Run, expect pass or diagnose**

Run: `./run_tests.sh`. If `test_same_seed_same_results` fails: the difference columns tell you which layer is non-deterministic. Common causes: a `randf()` call outside `rng`, iteration over a Dictionary whose insertion order differs, or a `_process` (not `_physics_process`) write. Fix the cause; do not relax the test.

- [ ] **Step 5: Implement experiment_config.gd, batch_runner.gd, batch.tscn, wrapper**

`scripts/experiment/experiment_config.gd`:

```gdscript
## A named set of RoundConfig variants to run against the same seeds.
class_name ExperimentConfig
extends Resource

@export var variants: Array[RoundConfig] = []
@export var rounds_per_variant: int = 20
```

`scripts/experiment/batch_runner.gd`:

```gdscript
## Headless experiment driver. Usage (from project root):
##   ./Godot_v4.7.2-stable_linux.x86_64 --headless --path . res://scenes/batch.tscn -- \
##       --config res://experiments/ablation.tres --rounds 50 --out experiments/ablation.csv
class_name BatchRunner
extends Node

@onready var gm: GameManager = $GameManager

var _variants: Array[RoundConfig] = []
var _rounds: int = 20
var _out_path: String = "experiments/results.csv"
var _log_transitions: bool = false
var _seed_offset: int = 0
var _time_scale: float = 8.0

var _vi: int = 0
var _si: int = 0
var _csv: FileAccess
var _tcsv: FileAccess
var _summary: Dictionary = {}  # variant -> {rounds, caught, total, duration_sum}


func _parse_args() -> bool:
	var args: PackedStringArray = OS.get_cmdline_user_args()
	var config_path: String = ""
	var i: int = 0
	while i < args.size():
		var a: String = args[i]
		var has_val: bool = i + 1 < args.size()
		match a:
			"--config":
				if not has_val: return false
				config_path = args[i + 1]; i += 1
			"--rounds":
				if not has_val: return false
				_rounds = int(args[i + 1]); i += 1
			"--out":
				if not has_val: return false
				_out_path = args[i + 1]; i += 1
			"--time-scale":
				if not has_val: return false
				_time_scale = float(args[i + 1]); i += 1
			"--seed-offset":
				if not has_val: return false
				_seed_offset = int(args[i + 1]); i += 1
			"--log-transitions":
				_log_transitions = true
			_:
				push_error("unknown arg %s" % a)
				return false
		i += 1
	if config_path == "":
		push_error("--config is required")
		return false
	var cfg := load(config_path) as ExperimentConfig
	if cfg == null:
		push_error("cannot load %s as ExperimentConfig" % config_path)
		return false
	_variants = cfg.variants
	if not args.has("--rounds"):
		_rounds = cfg.rounds_per_variant
	return not _variants.is_empty()


func _ready() -> void:
	if not _parse_args():
		get_tree().quit(2)
		return
	Engine.time_scale = _time_scale
	Engine.max_physics_steps_per_frame = maxi(8, int(ceil(_time_scale)) * 2)
	var abs_out: String = _out_path
	if _out_path.begins_with("res://") or _out_path.begins_with("user://"):
		abs_out = ProjectSettings.globalize_path(_out_path)
	elif not _out_path.is_absolute_path():
		abs_out = ProjectSettings.globalize_path("res://").path_join(_out_path)
	_csv = FileAccess.open(abs_out, FileAccess.WRITE)
	if _csv == null:
		push_error("cannot open %s" % abs_out)
		get_tree().quit(2)
		return
	_csv.store_csv_line(ResultsCsv.header())
	if _log_transitions:
		_tcsv = FileAccess.open(abs_out.get_basename() + "_transitions.csv", FileAccess.WRITE)
		_tcsv.store_csv_line(ResultsCsv.transitions_header())
	gm.round_finished.connect(_on_round_finished)
	_start_next()


func _start_next() -> void:
	if _vi >= _variants.size():
		_finish()
		return
	var cfg: RoundConfig = _variants[_vi].duplicate()
	cfg.seed = _seed_offset + _si
	print("[batch] variant=%s seed=%d (%d/%d)" % [cfg.variant_name, cfg.seed, _si + 1, _rounds])
	gm.start_round(cfg)


func _on_round_finished(res: Dictionary) -> void:
	_csv.store_csv_line(ResultsCsv.row(res))
	_csv.flush()
	if _tcsv:
		for r in ResultsCsv.transitions_rows(res):
			_tcsv.store_csv_line(r)
		_tcsv.flush()
	var v: String = res["variant"]
	if not _summary.has(v):
		_summary[v] = {"rounds": 0, "caught": 0, "total": 0, "duration_sum": 0.0}
	_summary[v]["rounds"] += 1
	_summary[v]["caught"] += int(res["hiders_caught"])
	_summary[v]["total"] += int(res["hiders_total"])
	_summary[v]["duration_sum"] += float(res["duration"])
	_si += 1
	if _si >= _rounds:
		_si = 0
		_vi += 1
	call_deferred("_start_next")


func _finish() -> void:
	_csv.close()
	if _tcsv:
		_tcsv.close()
	print("\n%-24s %6s %10s %10s" % ["variant", "rounds", "catch_rate", "mean_dur"])
	for v in _summary:
		var s: Dictionary = _summary[v]
		print("%-24s %6d %10.3f %10.1f" % [v, s["rounds"], float(s["caught"]) / maxi(1, s["total"]), s["duration_sum"] / maxi(1, s["rounds"])])
	print("[batch] wrote %s" % _out_path)
	get_tree().quit(0)
```

`scenes/batch.tscn`:

```
[gd_scene load_steps=4 format=3]

[ext_resource type="Script" path="res://scripts/experiment/batch_runner.gd" id="runner"]
[ext_resource type="PackedScene" path="res://scenes/arena/arena.tscn" id="arena"]
[ext_resource type="Script" path="res://scripts/game/game_manager.gd" id="gm"]

[node name="Batch" type="Node" ]
script = ExtResource("runner")

[node name="Arena" parent="." instance=ExtResource("arena")]

[node name="GameManager" type="Node" parent="."]
script = ExtResource("gm")
arena_path = NodePath("../Arena")
```

`run_experiment.sh`:

```bash
#!/usr/bin/env bash
# Usage: ./run_experiment.sh res://experiments/ablation.tres 50 experiments/ablation.csv [--log-transitions] [--time-scale 8]
set -eu
cd "$(dirname "$0")"
CONFIG=$1; ROUNDS=$2; OUT=$3; shift 3
./Godot_v4.7.2-stable_linux.x86_64 --headless --path . res://scenes/batch.tscn -- \
  --config "$CONFIG" --rounds "$ROUNDS" --out "$OUT" "$@"
```

`chmod +x run_experiment.sh`.

- [ ] **Step 6: Experiment configs**

`experiments/ablation.tres` (five seeker-sensor variants, identical otherwise):

```
[gd_resource type="Resource" script_class="ExperimentConfig" load_steps=8 format=3]

[ext_resource type="Script" path="res://scripts/experiment/experiment_config.gd" id="exp"]
[ext_resource type="Script" path="res://scripts/game/round_config.gd" id="cfg"]

[sub_resource type="Resource" id="all"]
script = ExtResource("cfg")
variant_name = "all_sensors"
hider_count = 3
head_start = 15.0
round_time = 120.0
seeker_sensors = PackedStringArray("lidar", "vision", "ultrasonic", "bumper")

[sub_resource type="Resource" id="no_vision"]
script = ExtResource("cfg")
variant_name = "no_vision"
hider_count = 3
head_start = 15.0
round_time = 120.0
seeker_sensors = PackedStringArray("lidar", "ultrasonic", "bumper")

[sub_resource type="Resource" id="no_lidar"]
script = ExtResource("cfg")
variant_name = "no_lidar"
hider_count = 3
head_start = 15.0
round_time = 120.0
seeker_sensors = PackedStringArray("vision", "ultrasonic", "bumper")

[sub_resource type="Resource" id="no_ultrasonic_bumper"]
script = ExtResource("cfg")
variant_name = "no_ultrasonic_bumper"
hider_count = 3
head_start = 15.0
round_time = 120.0
seeker_sensors = PackedStringArray("lidar", "vision")

[sub_resource type="Resource" id="vision_only"]
script = ExtResource("cfg")
variant_name = "vision_only"
hider_count = 3
head_start = 15.0
round_time = 120.0
seeker_sensors = PackedStringArray("vision")

[resource]
script = ExtResource("exp")
variants = Array[ExtResource("cfg")]([SubResource("all"), SubResource("no_vision"), SubResource("no_lidar"), SubResource("no_ultrasonic_bumper"), SubResource("vision_only")])
rounds_per_variant = 30
```

`experiments/strategies.tres`: same structure with two variants, `static_hiders` (`hider_strategy = "static"`) and `dynamic_hiders` (`hider_strategy = "dynamic"`), all sensors on, `rounds_per_variant = 30`.

If Godot refuses the typed `Array[ExtResource(...)]` syntax on load, write it as the editor would: create the resource once through the editor's Inspector (New Resource → ExperimentConfig, add variants) and copy the resulting `.tres` text; commit that form.

- [ ] **Step 7: Analysis script**

`experiments/analyze.py` (run with `python3 -I experiments/analyze.py experiments/ablation.csv`):

```python
#!/usr/bin/env python3
"""Summarise a batch CSV: catch rate, mean catch time, seeker distance per variant.
Writes <input>_summary.csv and <input>_catch_rate.png next to the input."""
import csv
import statistics
import sys
from collections import defaultdict
from pathlib import Path


def main(path: str) -> None:
    src = Path(path)
    groups: dict[str, list[dict]] = defaultdict(list)
    with src.open(newline="") as f:
        for row in csv.DictReader(f):
            groups[row["variant"]].append(row)

    rows = []
    for variant, rs in groups.items():
        total = sum(int(r["hiders_total"]) for r in rs)
        caught = sum(int(r["hiders_caught"]) for r in rs)
        catch_times = []
        for r in rs:
            for i in range(3):
                v = r.get(f"hider{i}_catch_time", "")
                if v not in ("", "-1.000"):
                    catch_times.append(float(v))
        rows.append({
            "variant": variant,
            "rounds": len(rs),
            "catch_rate": caught / total if total else 0.0,
            "mean_catch_time": statistics.mean(catch_times) if catch_times else float("nan"),
            "mean_seeker_distance": statistics.mean(float(r["seeker_distance"]) for r in rs),
            "mean_seeker_collisions": statistics.mean(float(r["seeker_collisions"]) for r in rs),
            "mean_seeker_stuck": statistics.mean(float(r["seeker_stuck"]) for r in rs),
        })

    out = src.with_name(src.stem + "_summary.csv")
    with out.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    for r in rows:
        print(f"{r['variant']:<24} n={r['rounds']:<4} catch_rate={r['catch_rate']:.3f} "
              f"mean_catch_time={r['mean_catch_time']:.1f} dist={r['mean_seeker_distance']:.1f}")

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(7, 3.5))
        ax.bar([r["variant"] for r in rows], [r["catch_rate"] for r in rows])
        ax.set_ylabel("catch rate")
        ax.set_ylim(0, 1)
        plt.xticks(rotation=20, ha="right")
        fig.tight_layout()
        fig.savefig(src.with_name(src.stem + "_catch_rate.png"), dpi=150)
    except ImportError:
        print("matplotlib not installed; skipped plot")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit("usage: analyze.py <results.csv>")
    main(sys.argv[1])
```

- [ ] **Step 8: Headless verification run**

```bash
./run_experiment.sh res://experiments/ablation.tres 2 experiments/smoke.csv --log-transitions --time-scale 8
python3 -I experiments/analyze.py experiments/smoke.csv
```

Expected: 10 rounds (5 variants × 2 seeds), exit code 0, `experiments/smoke.csv` has 11 lines, `experiments/smoke_transitions.csv` exists, summary table printed. Wall-clock for the run must be under 5 minutes (10 rounds × 135 s simulated at 8×). Measure with `time`; if it is slower, check that `Engine.max_physics_steps_per_frame` took effect and that the process is not render-bound. Delete `experiments/smoke*.csv` before committing, add `experiments/*.csv` and `experiments/*.png` to `.gitignore`.

- [ ] **Step 9: Commit**

```bash
./run_tests.sh
git add scripts/experiment scenes/batch.tscn experiments/*.tres experiments/analyze.py run_experiment.sh .gitignore tests/
git commit -m "feat: headless batch runner with CSV export and analysis script

Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>"
```

---

### Task 19: End-to-end verification and handoff

**Files:**
- Create: `docs/verification/2026-10-XX-first-full-run.md` (date of the run)
- Modify: `NOTES/INDEX.md`, new note `NOTES/tuning-log.md` only if parameter changes were needed during verification

This task produces evidence, not features. Nothing is "done" until the user has watched a round.

- [ ] **Step 1: Full test suite**

Run: `./run_tests.sh` → exit 0. Paste the final statistics line into the verification doc.

- [ ] **Step 2: Headless batch sanity**

```bash
time ./run_experiment.sh res://experiments/ablation.tres 5 experiments/ablation_smoke.csv
python3 -I experiments/analyze.py experiments/ablation_smoke.csv
```

Record in the verification doc: wall-clock time, the summary table, and whether `all_sensors` has a higher catch rate than `vision_only` and `no_vision` (expected but not guaranteed at n = 5; note what you see).

- [ ] **Step 3: Determinism spot check**

```bash
./run_experiment.sh res://experiments/strategies.tres 2 experiments/det_a.csv
./run_experiment.sh res://experiments/strategies.tres 2 experiments/det_b.csv
diff experiments/det_a.csv experiments/det_b.csv && echo IDENTICAL
```

Expected: `IDENTICAL`. Record the outcome.

- [ ] **Step 4: Visual run via computer-use-vnc**

Run the main scene (F5) and observe one full round with the overlay on. Check each FSM arrow at least once and tick it in the verification doc with the approximate time it was observed:

- Seeker: Wait→Patrol at 15 s; Patrol→Chase on a sighting; Chase→Catch (hider greys out); Catch→Patrol; Chase→Investigate after losing a hider; Investigate→Patrol timeout.
- Hider: PickSpot→GoToSpot→Hide; Hide→Flee when seen; Flee→PickSpot after escaping; Hide→Relocate on a lidar blob (watch a hider move when the seeker approaches unseen).
- Motion: a robot routes around a crate; a bumper recovery (reverse-and-turn) after a collision; no robot spins in place for more than 3 s.

Take screenshots of a chase and a catch into `docs/screenshots/`. Anything not observed goes in a "not yet observed" list, not a pass.

- [ ] **Step 5: Write the verification doc and commit**

`docs/verification/<date>-first-full-run.md` with sections: Test suite, Headless batch, Determinism, Visual run checklist, Not yet observed, Tuning changes made. Commit:

```bash
git add docs/verification docs/screenshots NOTES
git commit -m "docs: first end-to-end verification run

Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>"
```

- [ ] **Step 6: Hand over to the user**

Present the verification doc and ask the user to watch a live round themselves. Do not offer to merge or call the feature done until they confirm it behaves as hide-and-seek. Their confirmation closes this plan; tuning requests become new bounded tasks.
