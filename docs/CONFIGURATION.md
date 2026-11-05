# Frozen CL experimental configuration (September 2026)

These values are transcribed from the user's final CL technical report and Week 5 report. They are **recorded final values**, not a live dump of the currently installed robot.

| Group | Final reported configuration |
|---|---|
| Global planner | `global_planner/GlobalPlanner`; `use_dijkstra=true`, `use_grid_path=false`, `use_quadratic=false`, `publish_potential=false` |
| Global planning costs | `allow_unknown=true`, `default_tolerance=0.0`, `cost_factor=3.0`, `neutral_cost=50`, `lethal_cost=253` |
| Local planner | `teb_local_planner/TebLocalPlannerROS` |
| Motion limits | Forward 0.30 m/s, reverse 0.15 m/s, angular 1.20 rad/s; `penalty_epsilon=0.05` |
| Robot geometry | Circular radius 0.12 m; local padding 0.01 m; TEB circular model radius 0.12 m |
| Camera scan | 21 rays from -0.35 to +0.35 rad; 10 Hz; assumed blocked distance 0.40 m |
| Camera marks | `mark_radius=0.0`, `transition_only=false` in both layers |
| Global camera layer | `obstacle_ttl=15.0`, `preserve_on_reset=true` |
| Local camera layer | `obstacle_ttl=5.0`, `preserve_on_reset=false` |
| Freshness and clearing | `camera_timeout=1.0`, `scan_timeout=0.75`, `free_clear_hold=0.8`, `free_clear_range=0.45` |
| Inflation | Global radius 0.35 m / scaling 5.0; local radius 0.20 m / scaling 10.0 |
| Camera recovery | `max_duration=10`, `turn_speed=0.60`, `turn_margin=0.08`, `free_frames=5`, `free_handoff_delay=2.5` |

**Layer order:**

- Global: StaticLayer → ObstacleLayer → camera_global_layer (`ExpiringCameraLayer`) → InflationLayer.
- Local: StaticLayer → VoxelLayer → camera_obstacle_layer (`ExpiringCameraLayer`) → InflationLayer.

**Configuration source:** `cl_camera_plugins/launch/move_base_cl_v2.launch`. Review the recovered launch file against these values before publishing a claim that the posted source reproduces the final experiment. `rosparam` changes made at runtime do not necessarily persist; native C++ plugin changes require rebuilding and restarting the navigation process.

**Caution:** Speed values are configured limits, not directly measured robot travel speed, and the camera range is an assumption, not depth sensing.
