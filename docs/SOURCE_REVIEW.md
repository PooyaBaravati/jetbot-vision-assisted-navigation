# Source review: 5 November 2026

Review of the 27 KB source-only archive exported from the **JetBot** and **Ubuntu VM**, compared with the September 2026 CL technical and Week 4–5 progress reports.

## Verified contents

| Source | Primary files | Purpose |
| --- | --- | --- |
| `jetbot_ws/src/cl_camera_plugins` | `src/expiring_camera_layer.cpp`, `include/cl_camera_plugins/state.hpp` | Expiring camera-only marks and selective FREE clearing in local/global costmaps |
| Same package | `src/camera_turn_recovery.cpp` | Clearance-checked camera recovery integrated as a `move_base` recovery plugin |
| Same package | `launch/nav_cl_v2.launch`, `launch/move_base_cl_v2.launch` | Saved-map navigation and retained planner/costmap parameters |
| `vm_ws/src/jetbot_mission` | `scripts/ros_scene_perception.py` | ResNet18 blocked/free inference + Caffe MobileNet SSD labels/boxes |
| Same package | `scripts/camera_virtual_scan.py` | 21-ray 0.40 m *assumed* virtual LaserScan from stable classification |
| Same package | `launch/camera_navigation_system.launch` | **Final branch** perception + virtual scan |

## Distinguish historical code

- `vm_ws/src/jetbot_mission/scripts/camera_navigation_arbiter.py` and `camera_navigation_arbiter_before_full_scan.py` are historical **velocity-arbitration** implementations. They publish `/cmd_vel` and **must not** be launched concurrently with the final costmap-based branch.
- `vm_ws/src/jetbot_mission/launch/camera_navigation_system_with_arbiter.launch` is the **old** launch path; included for development history only.
- `jetbot_ws/src/cl_camera_plugins/scripts/configure.py` is a historical generator. Its embedded defaults (for example 30-second recovery, different camera TTLs and speeds) **differ from the retained final launch values**. Do not regenerate the final launch from it unless intentionally studying historical variants.
- `mission_manager.py`, `ros_collision_detector.py` and `ros_camera_viewer.py` are supplementary/earlier scripts and are **not** part of the final `camera_navigation_system.launch`.
- `test/state_test.cpp` was included in source; a complete automated test suite for the robot was not established.

## Potential reproducibility gaps

- `ros_scene_perception.py` uses a hard-coded absolute `/home/jetbot/catkin_ws/src/jetbot_mission` model directory. The combined node is the one launched by the final `.launch` file. It has **not** been patched or re-tested in this archived version.
- The compiled C++ depends on `jetbot_pro` YAML parameters and ROS navigation/plugin dependencies not included in this source-only export.
- Both the ONNX collision model and Caffe detector weights were deliberately left out; `deploy.prototxt` is included and requires upstream provenance/licence review.
- `jetbot_mission/package.xml` retains placeholder maintainer and `TODO` licence; `cl_camera_plugins/package.xml` declares BSD-3-Clause but uses a placeholder maintainer. No repository-wide licence or rights grant is assumed.
- The main launch file retains the final recorded settings: Dijkstra true, grid path false, quadratic false; global camera mark TTL 15 s, local TTL 5 s; maximum 10-second camera recoveries and broader global inflation.

## What was validated

During preparation **all Python scripts parsed successfully** and all ROS launch/plugin/manifest XML parsed. No C++ compile, neural model load, live ROS graph check, closed-loop test or cross-environment validation was possible in this file review.
