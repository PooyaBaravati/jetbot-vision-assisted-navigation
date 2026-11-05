# Model assets excluded from repository

The archived ROS scene perception node expects these files **on the Ubuntu VM**:

```text
/home/jetbot/catkin_ws/src/jetbot_mission/models/best_model_resnet18_opset9.onnx
/home/jetbot/catkin_ws/src/jetbot_mission/models/person_detector/deploy.prototxt
/home/jetbot/catkin_ws/src/jetbot_mission/models/person_detector/mobilenet_iter_73000.caffemodel
```

- The ResNet18 model was trained on a two-class (BLOCKED/FREE) lab dataset; it was exported to ONNX opset 9 for OpenCV 4.2 DNN.
- The MobileNet SSD detector uses the Caffe format and is only used for object labels/boxes in the final perception node; its bounding boxes do **not** directly supply navigation geometry.
- `deploy.prototxt` is included; the ONNX and `.caffemodel` weights are excluded pending size, ownership/licensing and privacy review.
- The trained checkpoint, conversion notebook and training dataset have not been provided in this source-only export.
- Preserve source model preprocessing and class order: index 0 is BLOCKED; index 1 is FREE.
