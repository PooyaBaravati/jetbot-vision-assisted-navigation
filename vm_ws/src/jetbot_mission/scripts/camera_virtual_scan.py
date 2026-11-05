#!/usr/bin/env python2
# -*- coding: utf-8 -*-
from __future__ import print_function

import time

import rospy

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String


class CameraVirtualScan(object):

    def __init__(self):
        rospy.init_node("camera_virtual_scan")

        self.frame_id = rospy.get_param(
            "~frame_id", "base_footprint"
        )

        self.obstacle_distance = rospy.get_param(
            "~obstacle_distance", 0.40
        )

        self.range_min = rospy.get_param(
            "~range_min", 0.05
        )

        self.range_max = rospy.get_param(
            "~range_max", 2.0
        )

        # Approximately 40-degree camera obstacle field.
        self.angle_min = rospy.get_param(
            "~angle_min", -0.35
        )

        self.angle_max = rospy.get_param(
            "~angle_max", 0.35
        )

        self.number_of_rays = rospy.get_param(
            "~number_of_rays", 21
        )

        self.publish_rate = rospy.get_param(
            "~publish_rate", 10.0
        )

        self.blocked_frames_required = rospy.get_param(
            "~blocked_frames", 5
        )

        self.free_frames_required = rospy.get_param(
            "~free_frames", 5
        )

        self.camera_timeout = rospy.get_param(
            "~camera_timeout", 1.0
        )

        self.blocked_count = 0
        self.free_count = 0

        # Fail-safe at startup until valid free data arrives.
        self.stable_blocked = True
        self.last_camera_message = 0.0
        self.last_state_text = ""

        self.scan_pub = rospy.Publisher(
            "/camera_virtual_scan",
            LaserScan,
            queue_size=1
        )

        self.state_pub = rospy.Publisher(
            "/camera_virtual_scan/state",
            String,
            queue_size=1,
            latch=True
        )

        rospy.Subscriber(
            "/camera/blocked",
            Bool,
            self.camera_callback,
            queue_size=1
        )

        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.publish_rate),
            self.publish_scan
        )

        rospy.loginfo("Camera virtual scan ready.")
        rospy.loginfo(
            "Virtual obstacle distance: %.2f m",
            self.obstacle_distance
        )
        rospy.loginfo(
            "Waiting for camera safety information."
        )

    def camera_callback(self, message):
        self.last_camera_message = time.time()

        if message.data:
            self.blocked_count = min(
                self.blocked_count + 1,
                100
            )
            self.free_count = 0

            if (
                self.blocked_count
                >= self.blocked_frames_required
            ):
                self.stable_blocked = True

        else:
            self.free_count = min(
                self.free_count + 1,
                100
            )
            self.blocked_count = 0

            if (
                self.free_count
                >= self.free_frames_required
            ):
                self.stable_blocked = False

    def publish_state(self, state_text):
        if state_text == self.last_state_text:
            return

        self.last_state_text = state_text

        self.state_pub.publish(
            String(data=state_text)
        )

        rospy.logwarn(
            "Camera virtual scan state: %s",
            state_text
        )

    def publish_scan(self, event):
        camera_stale = (
            time.time() - self.last_camera_message
            > self.camera_timeout
        )

        if camera_stale:
            obstacle_active = True
            self.publish_state("STALE_CAMERA_BLOCKED")

        elif self.stable_blocked:
            obstacle_active = True
            self.publish_state("VIRTUAL_OBSTACLE_ACTIVE")

        else:
            obstacle_active = False
            self.publish_state("CAMERA_PATH_FREE")

        scan = LaserScan()

        scan.header.stamp = rospy.Time.now()
        scan.header.frame_id = self.frame_id

        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max

        scan.angle_increment = (
            self.angle_max - self.angle_min
        ) / float(self.number_of_rays - 1)

        scan.time_increment = 0.0
        scan.scan_time = 1.0 / self.publish_rate

        scan.range_min = self.range_min
        scan.range_max = self.range_max

        if obstacle_active:
            scan.ranges = [
                self.obstacle_distance
                for unused_index
                in range(self.number_of_rays)
            ]
        else:
            # inf_is_valid allows these rays to clear only
            # the separate camera obstacle layer.
            scan.ranges = [
                float("inf")
                for unused_index
                in range(self.number_of_rays)
            ]

        scan.intensities = []

        self.scan_pub.publish(scan)


if __name__ == "__main__":
    try:
        CameraVirtualScan()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
