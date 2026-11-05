# Build and verification status

This repository preserves a **working-installation source snapshot** rather than a verified standalone installer. It targets **ROS 1 Melodic / Ubuntu 18.04**, not the newer ROS 2 Jazzy / Isaac Lab experiments.

## Included

- `jetbot_ws/src/cl_camera_plugins`: custom C++ `ExpiringCameraLayer`, `CameraTurnRecovery`, headers, plugin XML files, ROS launch files and `package.xml`/`CMakeLists.txt`.
- `vm_ws/src/jetbot_mission`: ROS Python perception and synthetic-scan scripts, ROS launch files, `deploy.prototxt`, package metadata.

## Not included; required to reproduce the tested setup

1. **Existing vendor `jetbot_pro` package** on JetBot and VM (or functional equivalent), including its differential-drive navigation, camera, localization and LiDAR launch dependencies.
2. **Local additions/config in `jetbot_pro`**, especially `config/diff/local_costmap_camera_params.yaml`, `config/diff/camera_costmap_params.yaml`, the other YAML paths referenced from `move_base_cl_v2.launch`, and a working `view_nav.launch`. These were not present in the uploaded source archive.
3. Saved lab map YAML **and its paired PGM image**, referenced as `week2_lab_map.yaml` in the tested command.
4. The trained classifier `best_model_resnet18_opset9.onnx` and the third-party detector `mobilenet_iter_73000.caffemodel` in the folders that `ros_scene_perception.py` expects. These binaries were excluded for review of rights, size and privacy.
5. ROS Melodic navigation stack with `move_base`, `global_planner`, `teb_local_planner`, `clear_costmap_recovery`, `costmap_2d`, TF, Python 2 `rospy`, OpenCV DNN and NumPy. See the ROS package manifests as a starting point; third-party packages are external dependencies.

## Copy to an existing development installation

**Do not overwrite your only working robot installation without backup.** The following is a *template* for an already configured JetBot/VM environment:

```bash
# On JetBot, from a clone of this repository:
cp -a jetbot_ws/src/cl_camera_plugins ~/catkin_ws/src/
cd ~/catkin_ws && catkin_make -j2 && source devel/setup.bash
rospack find cl_camera_plugins

# On Ubuntu VM, from a clone of this repository:
cp -a vm_ws/src/jetbot_mission ~/catkin_ws/src/
cd ~/catkin_ws && catkin_make && source devel/setup.bash
rospack find jetbot_mission
```

The copy commands assume the destination packages don't already exist; **back up or merge** them first if they do. Building and sourcing do not replace the missing map/models/vendor configuration.

## Important absolute model path

The original `vm_ws/src/jetbot_mission/scripts/ros_scene_perception.py` uses this literal path:

```text
/home/jetbot/catkin_ws/src/jetbot_mission
```

with expected assets:

```text
models/best_model_resnet18_opset9.onnx
models/person_detector/deploy.prototxt
models/person_detector/mobilenet_iter_73000.caffemodel
```

Copying the package to another home directory or a different workspace will require **editing this source path** (or changing the code to use `rospkg`/ROS parameters). The original working-source file has not been modified here.

## Syntax-only checks performed during repository preparation

- All Python source files successfully parsed with Python 3 AST syntax tools. This is **not** a Python 2 ROS runtime test.
- All included `.launch` and `.xml` files parsed as XML.
- The original C++ plugin files were inspected, but no native ROS Melodic compile and no robot-motion test were possible here.

A fresh ROS 1 Melodic build, end-to-end navigation trial, classification consistency check, parameter-server check and camera-to-actuation latency assessment are **not yet verified**.

## Start and stop

See [Runbook](RUNBOOK.md). On the JetBot use `cl_camera_plugins nav_cl_v2.launch`; on the VM use `jetbot_mission camera_navigation_system.launch` and `jetbot_pro view_nav.launch` in separate terminals. Do **not** start the old `camera_navigation_system_with_arbiter.launch` simultaneously with the final plugin navigation branch. Keep a safe manual stop available.
