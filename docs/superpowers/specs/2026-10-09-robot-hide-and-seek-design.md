# Robot Hide and Seek — Design Spec

Date: 2026-10-09
Status: approved in conversation, awaiting written review

## 1. Purpose and context

A 3D hide-and-seek game between autonomous robots, built in Godot 4.7.2, as a
university engineering project (see `Requirements.md`). Grading rewards:

- multiple decision-making algorithms (different actions under different conditions),
- use of as many robot sensors as possible,
- a self-built environment,
- experimental testing with measurable results (the heaviest-weighted report section).

The human only observes and configures. All robots are autonomous.

## 2. Decisions taken

| Topic | Decision |
|---|---|
| Control | Fully autonomous; human observes via camera and tweaks parameters |
| Robot model | Differential drive on a RigidBody3D, velocity set from the two-wheel unicycle model |
| Sensors | 2D lidar, vision cone, ultrasonic (x3), bumper. Noise configurable, zero in tests |
| World knowledge | Static wall layout known (navmesh); props and robots discovered only via sensors |
| Robots per round | 1 seeker vs 2–3 hiders (configurable) |
| Rules | Head start, timed seek phase, catch = within tag range + clear line of sight while seeker has detected the hider |
| Arena | Hand-built fixed walls, randomized seeded props per round |
| Decision logic | Hand-rolled finite state machines, one per role; hider strategy swappable |
| Experiments | Headless batch runner, CSV export; primary metrics are sensor ablation and catch/survival rate |
| Language / deps | GDScript; gdUnit4 for tests; no other addons |
| Scene authoring | Scenes written as text `.tscn`; editor used via computer-use-vnc for inspection, runs and screenshots |

Out of scope: sound sensors, odometry drift, multiple seekers, human control,
the written report itself.

## 3. Project structure

```
res://
  project.godot
  scenes/
    main.tscn            # Game root: Arena + GameManager + Camera rig + HUD
    batch.tscn           # Headless root: Arena + GameManager + BatchRunner (no camera/HUD)
    arena/arena.tscn     # Hand-built walls/floor, NavigationRegion3D, PropSpawner, spawn markers
    robots/robot.tscn    # Shared robot body (RigidBody3D chassis, wheels, sensor mount points)
    props/crate.tscn     # Randomized obstacle (StaticBody3D, a few size variants)
  scripts/
    robot/
      drive.gd           # Differential drive: wheel speeds -> body velocity in _integrate_forces
      sensors/           # lidar.gd, vision_cone.gd, ultrasonic.gd, bumper.gd (one node each)
      perception.gd      # Blackboard: latest sensor readings + derived facts
      motion.gd          # Path following + reactive avoidance -> (v, w) commands
      fsm/               # fsm.gd base, seeker_states.gd, hider_states.gd, hider_strategy.gd
      robot.gd           # Wires layers together, exposes role + strategy as exports
    game/
      game_manager.gd    # Round lifecycle, catch detection, timer, results record
      prop_spawner.gd    # Seeded randomized props, keeps spawn zones and corridors clear
      round_config.gd    # Resource: seed, hider count, timers, enabled sensors, strategy names
      visibility_grid.gd # Coarse LOS grid used by patrol ordering and hiding-spot scoring
    experiment/
      batch_runner.gd    # Headless: iterates variants x seeds, high time scale, writes CSV
    ui/
      hud.gd             # Timer, per-robot state labels, sensor debug overlay toggle
      camera_rig.gd      # Free orbit camera, follow-robot mode
  tests/                 # gdUnit4 tests
  experiments/           # Experiment configs, CSV outputs, Python analysis script
```

- One robot scene for both roles. Role and strategy are exported properties set
  by the game manager at spawn. Seeker is colour-coded but mechanically identical.
- Sensors are child nodes with `enabled` exports, toggled per round from the
  round config. Sensor ablation is a config change, not a code change.
- `RoundConfig` is a Resource; the same file drives editor runs and headless runs.
- Geometry comes from primitive meshes (BoxMesh, CylinderMesh) and CSG nodes.
  No external 3D assets.

## 4. Sensors

Each sensor is a Node3D child of the robot with `enabled` and `update_hz`
exports. It writes into the perception blackboard and reads the world only via
physics queries (raycasts, contact monitoring). Starting values below are
exported properties and will be tuned during development.

| Sensor | Model | Blackboard output |
|---|---|---|
| Lidar | Fan of 36 raycasts at 10° in the horizontal plane, range 8 m; hits walls, props, robots | `lidar_ranges: PackedFloat32Array`, derived `nearest_obstacle_dir`, `free_sectors` |
| Vision cone | FOV 90°, range 10 m; robots inside the cone get a LOS raycast to confirm | `seen_robots: Array` of {bearing, distance, role, time} |
| Ultrasonic | 3 cones (front, front-left, front-right), range 1.2 m; several rays per cone, minimum taken | `ultrasonic: [front, left, right]` |
| Bumper | Chassis contact monitoring, front arc only | `bumper_hit: bool`, `bumper_normal: Vector3` |

- Gaussian noise per sensor, std-dev exported, zero in tests.
- Perception derives role-specific facts: seeker keeps `last_known_hider_positions`;
  hiders keep `seeker_last_seen_pos`, `time_since_seeker_seen`.
- Disabled sensor leaves its fields empty. Decision layer has explicit fallbacks
  (e.g. without vision the seeker can only "detect" via a lidar blob within tag
  range). Those fallbacks are what the ablation experiment measures.
- Debug overlay draws rays and cone via ImmediateMesh when toggled.

## 5. Decision layer (FSMs)

Hand-rolled FSM base. A state has `enter(bb)`, `update(bb, dt) -> String|null`,
`exit(bb)`. States are plain GDScript classes with no node dependencies, read
only the blackboard, and write a goal (`goal_pos`, `goal_mode`) for the motion
layer. Every transition is logged with round time and reason.

### Seeker

```
Wait ──head start over──▶ Patrol ◀──────────────────────────┐
                            │ hider seen        │ lost for >3 s
                            ▼                   │
                          Chase ────────────────┘
                            │ in tag range + LOS
                            ▼
                          Catch (brief) ──▶ Patrol
Patrol ──partial glimpse / moving lidar blob──▶ Investigate ──nothing found──▶ Patrol
```

- Patrol: visits waypoints chosen as least-recently-viewed cells of the
  visibility grid.
- Investigate: goes to a last-known position after a brief sighting or a moving
  lidar blob; times out back to Patrol.
- Chase: pursues the hider's current or short-horizon predicted position; loses
  it after an unseen timeout.
- Catch: asks the game manager to mark the hider caught, then returns to Patrol.

### Hider

```
PickSpot ──spot chosen──▶ GoToSpot ──arrived──▶ Hide
Hide ──seeker seen──▶ Flee ──seeker unseen >4 s──▶ PickSpot
Hide ──seeker approaching but unseen (lidar blob)──▶ Relocate ──▶ GoToSpot
```

Hider strategy is a swappable object scoring candidate spots:

- `StaticHider`: picks the spot with lowest visibility from seeker spawn; stays unless seen.
- `DynamicHider`: periodically relocates to spots far from seeker's last known position.

Transitions use hysteresis (timeouts, thresholds with margins) to prevent
flicker.

## 6. Motion and drive

### Motion layer (goal -> (v, ω))

1. Global path from NavigationServer3D over the baked static map (walls only).
   Re-plan on goal change or stalled progress.
2. Local steering toward the next path point blended with a reactive lidar term
   that pushes heading away from close ranges, weighted by proximity. Handles
   props unknown to the navmesh.
3. Close-range override: ultrasonic below threshold scales down forward speed;
   bumper hit triggers a short reverse-and-turn recovery.
4. Stuck detection: progress toward goal below threshold for a few seconds
   raises `stuck` on the blackboard; FSM re-plans or re-picks goal. Stuck
   events are counted.

Output is clamped (v, ω).

### Drive layer (differential drive)

- (v, ω) -> wheel speeds: `wl = (v - ω·L/2)/r`, `wr = (v + ω·L/2)/r`; each
  clamped to `max_wheel_speed`, then rate-limited by `max_wheel_accel`.
- In `_integrate_forces`: forward linear velocity `r(wl+wr)/2` along the body
  axis, angular velocity `r(wr-wl)/L` about vertical. Gravity and collisions
  stay physical.
- Wheel meshes rotate visually at commanded speed.
- Explicit modelling assumption, stated in the report: no wheel slip or friction model.

## 7. Game rules and arena

### Round lifecycle (GameManager)

1. Setup: load `RoundConfig`, seed the single RNG, spawn props, bake navmesh
   (walls only) by script, spawn seeker and hiders at markers, apply sensor
   flags and strategies.
2. Head start (default 15 s): hiders move, seeker in Wait.
3. Seek phase (default 120 s): each physics frame check catch = seeker-to-hider
   distance < tag range (default 1.5 m) AND clear LOS ray AND seeker in Chase
   or Catch state. Caught hider stops and is greyed out.
4. End: all caught or time up. Emit results record: per hider caught/survived
   and catch time; per robot distance travelled, collisions, stuck events,
   state transition counts.

### Arena (~30 x 30 m)

- Floor, outer walls, fixed interior partial walls and pillars forming 3–4 rooms
  with multiple openings. BoxMesh + StaticBody3D in `arena.tscn`.
- Spawn markers: 1 seeker, 5 hider markers (random subset used).
- Prop spawner: N crates (default 12, varied sizes) at seeded positions inside
  clear zones; rejects overlap with walls, markers, other props; keeps corridors
  ≥ 1.5 m.
- Visibility grid: coarse grid, LOS sampled at setup against walls; props
  included only for robots that have observed them.

### Camera

Free orbit camera plus follow-robot mode, for demos and screenshots.

## 8. Experiment runner

Invocation:

```
./Godot_v4.7.2-stable_linux.x86_64 --headless --path . res://scenes/batch.tscn \
  -- --config experiments/ablation.tres --rounds 50 --out experiments/ablation.csv [--log-transitions]
```

- Experiment config lists variants (each a `RoundConfig` plus a name), e.g.
  all sensors / no vision / no lidar / static hiders / dynamic hiders.
- For each variant, run rounds with seeds 0..N-1 so every variant sees the same
  prop layouts.
- `Engine.time_scale` high (default 8), physics tick rate unchanged.
- One CSV row per round: variant, seed, strategy, enabled sensors, hiders caught,
  hiders total, catch times, seeker distance/collisions/stuck, per-hider same stats.
- Optional second CSV of FSM transitions.
- Summary table to stdout.
- Analysis (tables, plots) is a Python script in `experiments/`, outside Godot.
- Determinism: all randomness goes through one seeded RNG; same seed must yield
  identical CSV rows. This is a test.

## 9. Testing and verification

1. Unit tests (gdUnit4, headless CLI): drive math, every FSM transition against
   a fake blackboard incl. hysteresis, spot scoring and patrol ordering on a
   hand-made grid, prop spawner constraints and seed reproducibility, CSV formatting.
2. Scene tests (gdUnit4 scene runner, tiny test arena): lidar ranges against a
   wall at known distance; vision cone sees/doesn't see through walls; bumper
   and ultrasonic fire near a crate; robot moves forward / turns in place on command.
3. Real verification: rounds run in the editor via computer-use-vnc with
   screenshots, behaviour compared against the FSM diagrams; headless batch
   produces CSV and repeats a seed identically; the user watches a demo round
   and confirms it before anything is considered done or offered for merge.

## 10. Development order

Bottom-up so each layer is verifiable alone:
drive → arena + props → sensors + debug overlay → motion → FSMs → game rules →
batch runner. A visible robot driving in the arena comes early to de-risk the
editor-via-VNC workflow.
