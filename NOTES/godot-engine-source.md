# Godot engine source reference

- Local clone of the Godot engine source: `/mnt/LinuxGames/Repositories/godot`
  (cloned 2026-10-09; was still in progress when this note was written).
- Project runs on the bundled binary `Godot_v4.7.2-stable_linux.x86_64`
  (4.7.2.stable.official, Jolt Physics). Check out the matching tag
  `4.7.2-stable` in the clone before trusting an API detail.
- Use it to confirm exact engine behaviour that the docs leave vague, for
  example `RigidBody3D._integrate_forces` under Jolt, NavigationServer3D baking
  by script, or headless command-line argument handling.
- Search only inside that directory (shallow, targeted `grep`), never from a
  mount root.
