# Development history

## Week 1 — 24–28 August 2026

Assembly and ROS Melodic commissioning of the JetBot, SSH/VM setup, teleoperation, LiDAR, CSI camera, odometry and TF. Initial calibration recorded in the platform launch files.

## Week 2 — 31 August–4 September 2026

GMapping map creation and reload, AMCL localization, RViz goals, multi-point patrol and obstacle response with conventional LiDAR navigation. The nine provided ROS–OpenCV tutorials were exercised. Raw remote image transmission was a bottleneck; compressed transport performed substantially better.

## Week 3 — 7–11 September 2026

A ResNet18 BLOCKED/FREE classifier was trained using 159 images per class after dataset cleanup. A TensorRT notebook demonstration ran on the JetBot; SSD MobileNet V2 was also tested for person detection and a separate person-following demonstration. That person-following controller was not the final CL architecture.

## Week 4 — 14–18 September 2026

The collision model was exported to ONNX opset 9 and deployed through OpenCV DNN on the Ubuntu VM. The VM also ran MobileNet SSD for object boxes. An initial camera-navigation velocity arbiter could trigger turning but did not reliably force the global planner to reroute through an open direction.

## Week 5 — 21–25 September 2026

CL was changed to a costmap-based architecture with custom `ExpiringCameraLayer` and `CameraTurnRecovery` plugins. The experiments explored expiring camera marks, selective FREE clearing, global and local obstacle layer persistence, recovery timing and GlobalPlanner settings. A final qualitative configuration was retained as an experimental baseline, not a validated robust navigation product.

**Note:** The Week 4 report and four-week summary describe intermediate architectures and should not be cited as the final configuration. The Week 5 report and the CL technical report record the later plugin-era baseline.
