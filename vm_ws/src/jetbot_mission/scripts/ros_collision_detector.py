#!/usr/bin/env python

from __future__ import print_function

import time

import cv2
import numpy as np
import rospy

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool
from std_msgs.msg import Float32


class RosCollisionDetector(object):

    def __init__(self):
        default_model = (
            "/home/jetbot/catkin_ws/src/"
            "jetbot_mission/models/"
            "best_model_resnet18_opset9.onnx"
        )

        self.image_topic = rospy.get_param(
            "~image_topic",
            "/csi_cam_0/image_raw/compressed"
        )

        self.model_path = rospy.get_param(
            "~model_path",
            default_model
        )

        self.block_threshold = rospy.get_param(
            "~block_threshold",
            0.60
        )

        self.clear_threshold = rospy.get_param(
            "~clear_threshold",
            0.40
        )

        self.blocked = False
        self.previous_time = None
        self.filtered_fps = 0.0

        rospy.loginfo(
            "Loading collision model: %s",
            self.model_path
        )

        self.net = cv2.dnn.readNetFromONNX(
            self.model_path
        )

        self.probability_pub = rospy.Publisher(
            "/camera/blocked_probability",
            Float32,
            queue_size=1
        )

        self.blocked_pub = rospy.Publisher(
            "/camera/blocked",
            Bool,
            queue_size=1
        )

        rospy.Subscriber(
            self.image_topic,
            CompressedImage,
            self.image_callback,
            queue_size=1,
            buff_size=16777216
        )

        rospy.on_shutdown(self.shutdown)

        rospy.loginfo(
            "Collision detector subscribed to %s",
            self.image_topic
        )

        rospy.loginfo(
            "Blocked threshold: %.2f",
            self.block_threshold
        )

        rospy.loginfo(
            "Clear threshold: %.2f",
            self.clear_threshold
        )

    def preprocess(self, frame):
        resized = cv2.resize(
            frame,
            (224, 224)
        )

        rgb = cv2.cvtColor(
            resized,
            cv2.COLOR_BGR2RGB
        )

        image = rgb.astype(
            np.float32
        ) / 255.0

        mean = np.array(
            [0.485, 0.456, 0.406],
            dtype=np.float32
        )

        std = np.array(
            [0.229, 0.224, 0.225],
            dtype=np.float32
        )

        image = (
            image - mean
        ) / std

        image = np.transpose(
            image,
            (2, 0, 1)
        )

        blob = np.expand_dims(
            image,
            axis=0
        )

        return np.ascontiguousarray(
            blob,
            dtype=np.float32
        )

    def softmax(self, logits):
        logits = logits - np.max(logits)

        values = np.exp(logits)

        return values / np.sum(values)

    def calculate_fps(self):
        current_time = time.time()

        if self.previous_time is None:
            self.previous_time = current_time
            return 0.0

        elapsed = current_time - self.previous_time
        self.previous_time = current_time

        if elapsed <= 0:
            return self.filtered_fps

        current_fps = 1.0 / elapsed

        if self.filtered_fps == 0:
            self.filtered_fps = current_fps
        else:
            self.filtered_fps = (
                0.9 * self.filtered_fps +
                0.1 * current_fps
            )

        return self.filtered_fps

    def image_callback(self, message):
        encoded = np.frombuffer(
            message.data,
            dtype=np.uint8
        )

        frame = cv2.imdecode(
            encoded,
            cv2.IMREAD_COLOR
        )

        if frame is None:
            rospy.logwarn_throttle(
                2.0,
                "Could not decode camera frame."
            )
            return

        blob = self.preprocess(frame)

        self.net.setInput(blob)

        logits = self.net.forward().reshape(-1)

        probabilities = self.softmax(logits)

        # ImageFolder sorted the classes:
        # index 0 = blocked
        # index 1 = free
        blocked_probability = float(
            probabilities[0]
        )

        if blocked_probability >= self.block_threshold:
            self.blocked = True

        elif blocked_probability <= self.clear_threshold:
            self.blocked = False

        self.probability_pub.publish(
            Float32(
                data=blocked_probability
            )
        )

        self.blocked_pub.publish(
            Bool(
                data=self.blocked
            )
        )

        fps = self.calculate_fps()

        if self.blocked:
            state_text = "BLOCKED"
            colour = (0, 0, 255)
        else:
            state_text = "FREE"
            colour = (0, 255, 0)

        cv2.putText(
            frame,
            "{} | probability: {:.2f}".format(
                state_text,
                blocked_probability
            ),
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            colour,
            2
        )

        cv2.putText(
            frame,
            "Inference FPS: {:.1f}".format(fps),
            (10, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            colour,
            2
        )

        cv2.imshow(
            "JetBot ROS Collision Detector",
            frame
        )

        if cv2.waitKey(1) & 0xFF == ord("q"):
            rospy.signal_shutdown(
                "Detector closed by user."
            )

    def shutdown(self):
        cv2.destroyAllWindows()


if __name__ == "__main__":
    rospy.init_node(
        "jetbot_ros_collision_detector"
    )

    detector = RosCollisionDetector()

    rospy.spin()
