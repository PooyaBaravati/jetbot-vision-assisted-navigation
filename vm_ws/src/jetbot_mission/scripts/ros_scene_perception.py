#!/usr/bin/env python2
from __future__ import print_function

import time

import cv2
import numpy as np
import rospy

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, Float32, String


class RosScenePerception(object):

    def __init__(self):
        rospy.init_node("ros_scene_perception")

        package_path = (
            "/home/jetbot/catkin_ws/src/jetbot_mission"
        )

        collision_model_path = (
            package_path
            + "/models/best_model_resnet18_opset9.onnx"
        )

        object_model_folder = (
            package_path
            + "/models/person_detector/"
        )

        object_prototxt_path = (
            object_model_folder + "deploy.prototxt"
        )

        object_model_path = (
            object_model_folder
            + "mobilenet_iter_73000.caffemodel"
        )

        self.blocked_threshold = rospy.get_param(
            "~blocked_threshold", 0.70
        )

        self.clear_threshold = rospy.get_param(
            "~clear_threshold", 0.40
        )

        self.object_confidence_threshold = rospy.get_param(
            "~object_confidence_threshold", 0.50
        )

        # Run object detection every third camera frame.
        # Collision classification still runs on every frame.
        self.object_every_n_frames = rospy.get_param(
            "~object_every_n_frames", 3
        )

        self.display_width = rospy.get_param(
            "~display_width", 900
        )

        self.voc_classes = [
            "background",
            "aeroplane",
            "bicycle",
            "bird",
            "boat",
            "bottle",
            "bus",
            "car",
            "cat",
            "chair",
            "cow",
            "diningtable",
            "dog",
            "horse",
            "motorbike",
            "person",
            "pottedplant",
            "sheep",
            "sofa",
            "train",
            "tvmonitor"
        ]

        rospy.loginfo("Loading collision model...")

        self.collision_net = cv2.dnn.readNetFromONNX(
            collision_model_path
        )

        rospy.loginfo("Collision model loaded.")

        rospy.loginfo("Loading object-detection model...")

        self.object_net = cv2.dnn.readNetFromCaffe(
            object_prototxt_path,
            object_model_path
        )

        rospy.loginfo("Object-detection model loaded.")

        self.blocked_probability_pub = rospy.Publisher(
            "/camera/blocked_probability",
            Float32,
            queue_size=1
        )

        self.free_probability_pub = rospy.Publisher(
            "/camera/free_probability",
            Float32,
            queue_size=1
        )

        self.blocked_pub = rospy.Publisher(
            "/camera/blocked",
            Bool,
            queue_size=1
        )

        self.detected_objects_pub = rospy.Publisher(
            "/camera/detected_objects",
            String,
            queue_size=1
        )

        self.is_blocked = False
        self.frame_counter = 0
        self.latest_objects = []

        self.previous_time = time.time()
        self.filtered_fps = 0.0

        self.camera_subscriber = rospy.Subscriber(
            "/csi_cam_0/image_raw/compressed",
            CompressedImage,
            self.image_callback,
            queue_size=1,
            buff_size=2 ** 24
        )

        rospy.on_shutdown(self.shutdown)

        rospy.loginfo(
            "Subscribed to /csi_cam_0/image_raw/compressed"
        )

        rospy.loginfo(
            "Blocked threshold: %.2f",
            self.blocked_threshold
        )

        rospy.loginfo(
            "Clear threshold: %.2f",
            self.clear_threshold
        )

        rospy.loginfo(
            "Object confidence threshold: %.2f",
            self.object_confidence_threshold
        )

        rospy.loginfo("Press Q in the camera window to stop.")

    def softmax(self, values):
        values = values.astype(np.float32)
        values = values - np.max(values)

        exponentials = np.exp(values)

        return exponentials / np.sum(exponentials)

    def run_collision_model(self, frame):
        resized = cv2.resize(frame, (224, 224))
        rgb_image = cv2.cvtColor(
            resized,
            cv2.COLOR_BGR2RGB
        )

        image = rgb_image.astype(np.float32) / 255.0

        mean = np.array(
            [0.485, 0.456, 0.406],
            dtype=np.float32
        )

        standard_deviation = np.array(
            [0.229, 0.224, 0.225],
            dtype=np.float32
        )

        image = (
            image - mean
        ) / standard_deviation

        image = np.transpose(image, (2, 0, 1))
        image = np.expand_dims(image, axis=0)

        self.collision_net.setInput(image)

        output = self.collision_net.forward().flatten()
        probabilities = self.softmax(output)

        # Dataset class order:
        # class 0 = blocked
        # class 1 = free
        blocked_probability = float(probabilities[0])
        free_probability = float(probabilities[1])

        return blocked_probability, free_probability

    def run_object_detector(self, frame):
        frame_height, frame_width = frame.shape[:2]

        blob = cv2.dnn.blobFromImage(
            frame,
            scalefactor=0.007843,
            size=(300, 300),
            mean=(127.5, 127.5, 127.5),
            swapRB=False,
            crop=False
        )

        self.object_net.setInput(blob)
        detections = self.object_net.forward()

        detected_objects = []

        for index in range(detections.shape[2]):
            confidence = float(
                detections[0, 0, index, 2]
            )

            if confidence < self.object_confidence_threshold:
                continue

            class_id = int(
                detections[0, 0, index, 1]
            )

            if class_id <= 0:
                continue

            if class_id >= len(self.voc_classes):
                continue

            coordinates = (
                detections[0, 0, index, 3:7]
                * np.array(
                    [
                        frame_width,
                        frame_height,
                        frame_width,
                        frame_height
                    ]
                )
            )

            x1, y1, x2, y2 = coordinates.astype("int")

            x1 = max(0, min(x1, frame_width - 1))
            y1 = max(0, min(y1, frame_height - 1))
            x2 = max(0, min(x2, frame_width - 1))
            y2 = max(0, min(y2, frame_height - 1))

            if x2 <= x1 or y2 <= y1:
                continue

            detected_objects.append(
                {
                    "label": self.voc_classes[class_id],
                    "confidence": confidence,
                    "x1": x1,
                    "y1": y1,
                    "x2": x2,
                    "y2": y2
                }
            )

        return detected_objects

    def update_blocked_state(self, blocked_probability):
        # Blocked only when model probability exceeds 70 percent.
        self.is_blocked = blocked_probability > 0.70

    def publish_results(
        self,
        blocked_probability,
        free_probability
    ):
        self.blocked_probability_pub.publish(
            Float32(data=blocked_probability)
        )

        self.free_probability_pub.publish(
            Float32(data=free_probability)
        )

        self.blocked_pub.publish(
            Bool(data=self.is_blocked)
        )

        if self.latest_objects:
            object_text = ", ".join(
                [
                    "{}:{:.2f}".format(
                        detected_object["label"],
                        detected_object["confidence"]
                    )
                    for detected_object in self.latest_objects
                ]
            )
        else:
            object_text = "none"

        self.detected_objects_pub.publish(
            String(data=object_text)
        )

    def draw_objects(self, frame):
        for detected_object in self.latest_objects:
            label = detected_object["label"]
            confidence = detected_object["confidence"]

            x1 = detected_object["x1"]
            y1 = detected_object["y1"]
            x2 = detected_object["x2"]
            y2 = detected_object["y2"]

            if label == "person":
                color = (0, 255, 0)
            else:
                color = (255, 180, 0)

            cv2.rectangle(
                frame,
                (x1, y1),
                (x2, y2),
                color,
                2
            )

            label_text = "{}: {:.2f}".format(
                label.upper(),
                confidence
            )

            text_y = max(25, y1 - 8)

            cv2.putText(
                frame,
                label_text,
                (x1, text_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.60,
                color,
                2
            )

    def draw_status(
        self,
        frame,
        blocked_probability,
        free_probability
    ):
        overlay = frame.copy()

        cv2.rectangle(
            overlay,
            (0, 0),
            (480, 125),
            (0, 0, 0),
            -1
        )

        cv2.addWeighted(
            overlay,
            0.55,
            frame,
            0.45,
            0,
            frame
        )

        if self.is_blocked:
            state_text = "CAMERA: BLOCKED"
            state_color = (0, 0, 255)
        else:
            state_text = "CAMERA: FREE"
            state_color = (0, 255, 0)

        cv2.putText(
            frame,
            state_text,
            (15, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            state_color,
            2
        )

        probability_text = (
            "Blocked: {:.2f} | Free: {:.2f}"
        ).format(
            blocked_probability,
            free_probability
        )

        cv2.putText(
            frame,
            probability_text,
            (15, 62),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 255),
            2
        )

        cv2.putText(
            frame,
            "Perception FPS: {:.1f}".format(
                self.filtered_fps
            ),
            (15, 94),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 255),
            2
        )

    def update_fps(self):
        current_time = time.time()
        elapsed = current_time - self.previous_time
        self.previous_time = current_time

        if elapsed <= 0:
            return

        current_fps = 1.0 / elapsed

        if self.filtered_fps == 0.0:
            self.filtered_fps = current_fps
        else:
            self.filtered_fps = (
                0.90 * self.filtered_fps
                + 0.10 * current_fps
            )

    def image_callback(self, message):
        try:
            compressed_data = np.fromstring(
                message.data,
                dtype=np.uint8
            )

            frame = cv2.imdecode(
                compressed_data,
                cv2.IMREAD_COLOR
            )

            if frame is None:
                rospy.logwarn_throttle(
                    2.0,
                    "Could not decode camera image."
                )
                return

            self.frame_counter += 1

            blocked_probability, free_probability = (
                self.run_collision_model(frame)
            )

            self.update_blocked_state(
                blocked_probability
            )

            run_object_detection = (
                self.frame_counter == 1
                or self.frame_counter
                % self.object_every_n_frames == 0
            )

            if run_object_detection:
                self.latest_objects = (
                    self.run_object_detector(frame)
                )

            self.publish_results(
                blocked_probability,
                free_probability
            )

            self.update_fps()
            self.draw_objects(frame)

            self.draw_status(
                frame,
                blocked_probability,
                free_probability
            )

            frame_height, frame_width = frame.shape[:2]

            if frame_width > self.display_width:
                display_height = int(
                    frame_height
                    * float(self.display_width)
                    / float(frame_width)
                )

                display_frame = cv2.resize(
                    frame,
                    (
                        self.display_width,
                        display_height
                    )
                )
            else:
                display_frame = frame

            cv2.imshow(
                "JetBot ROS Scene Perception",
                display_frame
            )

            key = cv2.waitKey(1) & 0xFF

            if key == ord("q"):
                rospy.signal_shutdown(
                    "Stopped from camera window."
                )

        except Exception as error:
            rospy.logerr_throttle(
                2.0,
                "Perception error: {}".format(error)
            )

    def shutdown(self):
        self.blocked_pub.publish(Bool(data=False))
        cv2.destroyAllWindows()
        rospy.loginfo("Scene-perception node stopped.")


if __name__ == "__main__":
    try:
        RosScenePerception()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
