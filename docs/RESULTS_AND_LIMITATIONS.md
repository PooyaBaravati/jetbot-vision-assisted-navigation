# Experimental results and limitations

## Supported observations

- Robot mapping, saved-map localization, and standard LiDAR goal navigation were demonstrated in the earlier laboratory work.
- ROS VM camera inference ran with an exported ONNX ResNet18 FREE/BLOCKED classifier and a separate MobileNet SSD object detector.
- The camera states were converted into temporary synthetic obstacle evidence and integrated with the local/global costmaps.
- Later costmap experiments were reported to influence global route selection in successful qualitative trials.
- The operator reported satisfactory navigation with Dijkstra expansion, gradient-based extraction and quadratic potential disabled.
- A VM allocation of four CPUs and approximately 8 GiB RAM was associated with a roughly 25 FPS perception display in one late test; FPS is **not** end-to-end response time.

## Unsupported claims — do not make these in a paper or résumé

- There is no completed controlled goal-success-rate or collision-rate benchmark for final CL.
- There is no confirmed final confusion matrix or held-out accuracy estimate for the classifier. A 90% reading displayed at one development epoch is not a final validated test result.
- There is no measured complete image-capture-to-wheel-command latency distribution.
- The system was not demonstrated to be safe or robust across different environments and obstacle types.
- Disabling quadratic global-planner calculation was associated with improved observed behavior, but was not isolated as a causal CPU-performance improvement.

## Important physical limitation

A binary image classification cannot locate the obstacle, estimate its geometry, or measure range. The virtual scan placed blocked marks at an assumed 0.40 m, regardless of true object distance. Repeated marks while turning can create fake barriers. Short mark lifetimes can cause the planner to forget a blocked passage; long lifetimes preserve both useful and false evidence.

## Future scientific evaluation

Compare three methods with identical start-goal pairs, environmental conditions, mapping and planner parameters:

1. LiDAR-only ROS navigation.
2. Frozen CL plugin-era baseline.
3. A candidate calibrated spatial-perception system (segmentation or depth-assisted obstacle mapping).

Measure goal completion, human interventions or collisions, recoveries, path length, time to goal, false obstacle events, camera-clearance timing, sensor-to-costmap and control latency, and robot/VM resource utilization. Run multiple trials and report variability. Spatial perception alternatives were **proposed**, not implemented in CL.
