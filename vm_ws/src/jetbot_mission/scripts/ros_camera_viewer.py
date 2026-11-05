#!/usr/bin/env python

from __future__ import print_function

import time

import cv2
import numpy as np
import rospy

from sensor_msgs.msg import CompressedImage


class RosCameraViewer(object):

    def __init__(self):
        self.previous_time = None
        self.filtered_fps = 0.0

        self.topic = rospy.get_param(
            "~image_topic",
            "/csi_cam_0/image_raw/compressed"
        )

        rospy.Subscriber(
            self.topic,
            CompressedImage,
            self.image_callback,
            queue_size=1,
            buff_size=16777216
        )

        rospy.on_shutdown(self.shutdown)

        rospy.loginfo(
            "Camera viewer subscribed to %s",
            self.topic
        )

        rospy.loginfo(
            "Press Q in the video window to stop."
        )

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
        encoded_image = np.frombuffer(
            message.data,
            dtype=np.uint8
        )

        frame = cv2.imdecode(
            encoded_image,
            cv2.IMREAD_COLOR
        )

        if frame is None:
            rospy.logwarn_throttle(
                2.0,
                "Could not decode camera frame."
            )
            return

        fps = self.calculate_fps()

        cv2.putText(
            frame,
            "ROS camera | FPS: {:.1f}".format(fps),
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2
        )

        cv2.imshow(
            "JetBot ROS Camera",
            frame
        )

        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
            rospy.signal_shutdown(
                "Viewer closed by user."
            )

    def shutdown(self):
        cv2.destroyAllWindows()


if __name__ == "__main__":
    rospy.init_node(
        "jetbot_ros_camera_viewer"
    )

    viewer = RosCameraViewer()

    rospy.spin()
