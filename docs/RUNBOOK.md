# Operating the final experimental CL setup

This runbook is adapted from `final.txt`, the CL technical report and the Week 5 report. It is for the **preconfigured** ROS Melodic JetBot and Ubuntu VM, not an independent installation recipe.

## Preparations

- Check that the JetBot and Ubuntu VM are on a network with bidirectional ROS connectivity.
- Verify the actual JetBot IP and VM IP; old addresses and hostnames may change.
- Use **one ROS master** and **one navigation launch**.
- Ensure the source packages, compiled C++ plugin, map YAML/PGM pair, pretrained model assets and correct class-label mapping are present.
- Ensure no old camera-navigation velocity arbiter is running.

## Terminal T1 — JetBot SSH

```bash
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch cl_camera_plugins nav_cl_v2.launch \
  map_file:=/home/jetbot/catkin_ws/src/jetbot_pro/maps/week2_lab_map.yaml
```

The user's original command note lists `roslaunch jetbot_pro nav_camera_costmap.launch` as an **alternative**. To document the final costmap-plugin experimental branch, prefer `cl_camera_plugins nav_cl_v2.launch`; do **not** launch both.

## Terminal T2 — Ubuntu VM perception

```bash
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://nano-4gb-jp45:11311  # use reachable robot hostname/IP
# Also configure a VM address reachable by the JetBot (ROS_IP or ROS_HOSTNAME).
roslaunch jetbot_mission camera_navigation_system.launch
```

## Terminal T3 — Ubuntu VM RViz

```bash
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://nano-4gb-jp45:11311  # use reachable robot hostname/IP
roslaunch jetbot_pro view_nav.launch
```

In RViz set **Fixed Frame** to `map`. Add:

- **LaserScan** topic `/camera_virtual_scan`
- **Map** topic `/move_base/local_costmap/costmap`

Use 2D Pose Estimate so the physical scan aligns with the saved map. Send a short goal only with a clear controlled test area, a human supervising the robot and a stop available.

## Diagnostics

```bash
rostopic info /cmd_vel
rostopic info /camera_virtual_scan
rostopic echo -n 1 /camera/blocked
rostopic echo -n 1 /camera_virtual_scan/state
rosparam get /move_base/GlobalPlanner
rosparam get /move_base/recovery_behaviors
rosparam get /move_base/global_costmap/inflation_layer
rosparam get /move_base/local_costmap/inflation_layer
```

Optional layer diagnostics:

```bash
rostopic echo -n 1 /move_base/global_costmap/camera_global_layer/active_mark_count
rostopic echo -n 1 /move_base/local_costmap/camera_obstacle_layer/active_mark_count
```

> A nonzero mark count does not verify that expected cells are lethal in the published master costmap or that global replanning truly avoided them.

## Cancel a goal

First cancel the active goal in RViz if possible. The original operator note used the following **ROS 1 actionlib cancel** command:

```bash
rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID \
  "{stamp: {secs: 0, nsecs: 0}, id: ''}"
```

This sends a cancellation request to the action server; it is not a hard hardware emergency stop. Verify that the robot stopped and that no other velocity publisher remains active. `rostopic echo /move_base/goal` is an inspection command, not a deletion command.

## Rebuild after C++ changes

```bash
# Stop navigation before rebuilding
cd ~/catkin_ws
catkin_make -j2
source devel/setup.bash
# Restart CL navigation so move_base loads the rebuilt plugins
```

## Model formats and availability

- ResNet18 checkpoint originated in the JetBot Jupyter environment and was exported to `best_model_resnet18_opset9.onnx` for OpenCV 4.2 compatibility.
- `ros_scene_perception.py` referenced the compatible ONNX model and the MobileNet SSD `deploy.prototxt` and `mobilenet_iter_73000.caffemodel` in the VM package.
- Keep model preprocessing, channel order, normalization and FREE/BLOCKED class mapping consistent with the training/export code.
- The repository includes the source packages but does not contain the trained model, raw dataset or vendor-owned detector weights.
