# Architecture and ROS message flow

This note documents the **final plugin-era CL architecture**, not the early arbiter proof of concept.

## VM perception and virtual scan

1. The JetBot publishes CSI camera frames using `/csi_cam_0/image_raw/compressed`.
2. `ros_scene_perception.py` consumes the images. A ResNet18 ONNX classifier estimates blocked/free view probabilities; MobileNet SSD adds scene labels and bounding boxes for interpretation. The detector's boxes were **not used as range measurements**.
3. The perception node reports camera state on `/camera/blocked`. The development defaults were 0.70 for blocked and 0.40 for clear to reduce switching.
4. `camera_virtual_scan.py` stabilizes state over five consecutive blocked/free reports, producing 21 synthetic ranges over ±0.35 rad at 10 Hz. A stable blocked state marks ranges at **0.40 m assumed distance**; a stable free state emits +inf ranges. Scan limits were 0.05–2.0 m. Camera message age greater than 1 s yields a stale blocked state.

## JetBot navigation stack

5. The custom `ExpiringCameraLayer` on both local and global costmaps transforms synthetic endpoints into costmap coordinates. Separate LiDAR obstacle layers continue to consume `/scan`.
6. A global camera layer permits tentative visual blockage to affect path search; a local camera layer influences trajectory feasibility. Inflation is applied after camera and LiDAR obstacle layers.
7. `move_base` uses `global_planner/GlobalPlanner` for global routes and `teb_local_planner/TebLocalPlannerROS` for local trajectories.
8. A `CameraTurnRecovery` plugin checks fresh sensor/costmap information and swept-turn clearance before commanding a recovery turn. It can wait instead of forcing a turn when clearance is uncertain.

## Why the architecture changed

The previous `camera_navigation_arbiter.py` intercepted planner velocity messages and selected LiDAR-informed turns, but this alone did not reliably affect the planned global route. The plugin-era architecture applies camera-derived temporary spatial costs directly to both costmaps.

> Do not run `camera_navigation_system_with_arbiter.launch` or the old arbiter alongside `nav_cl_v2.launch`. Multiple `/cmd_vel` publishers could issue competing motion commands.

## Observability

- `rostopic info /cmd_vel` checks command ownership.
- `/camera_virtual_scan/state` identifies active/free/stale states.
- `/move_base/*_costmap/*camera*_layer/active_mark_count` reports stored marks, **not** proof that the master costmap contains the intended lethal cells.
- Compare timestamped global paths, costmaps, TF and camera state when diagnosing unexpected replanning.
