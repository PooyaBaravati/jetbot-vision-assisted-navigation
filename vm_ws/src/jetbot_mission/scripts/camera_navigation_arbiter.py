#!/usr/bin/env python2
# -*- coding: utf-8 -*-
from __future__ import print_function

import math
import time

import rospy

from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, String


class CameraNavigationArbiter(object):

    NAVIGATION = "NAVIGATION"
    ROTATING = "CAMERA_RECOVERY"
    SAFETY_STOP = "SAFETY_STOP"

    def __init__(self):
        rospy.init_node("camera_navigation_arbiter")

        self.blocked_frames_required = rospy.get_param(
            "~blocked_frames", 5
        )

        self.free_frames_required = rospy.get_param(
            "~free_frames", 5
        )

        self.turning_speed = rospy.get_param(
            "~turning_speed", 0.60
        )

        self.minimum_sector_clearance = rospy.get_param(
            "~minimum_sector_clearance", 0.45
        )

        self.critical_rotation_clearance = rospy.get_param(
            "~critical_rotation_clearance", 0.22
        )

        self.camera_timeout = rospy.get_param(
            "~camera_timeout", 1.0
        )

        self.navigation_timeout = rospy.get_param(
            "~navigation_timeout", 0.50
        )

        self.state = self.NAVIGATION
        self.turn_direction = 0
        self.target_angle = 0.0

        self.blocked_count = 0
        self.free_count = 0
        self.goal_active = False

        self.latest_scan = None
        self.latest_navigation_command = Twist()

        self.last_camera_time = 0.0
        self.last_navigation_time = 0.0
        self.last_retry_time = 0.0

        self.velocity_pub = rospy.Publisher(
            "/cmd_vel",
            Twist,
            queue_size=1
        )

        self.state_pub = rospy.Publisher(
            "/camera_navigation/state",
            String,
            queue_size=1,
            latch=True
        )

        self.target_angle_pub = rospy.Publisher(
            "/camera_navigation/target_angle",
            Float32,
            queue_size=1,
            latch=True
        )

        rospy.Subscriber(
            "/cmd_vel_nav",
            Twist,
            self.navigation_callback,
            queue_size=1
        )

        rospy.Subscriber(
            "/camera/blocked",
            Bool,
            self.camera_callback,
            queue_size=1
        )

        rospy.Subscriber(
            "/scan",
            LaserScan,
            self.scan_callback,
            queue_size=1
        )

        rospy.Subscriber(
            "/move_base/status",
            GoalStatusArray,
            self.status_callback,
            queue_size=1
        )

        self.control_timer = rospy.Timer(
            rospy.Duration(0.05),
            self.control_loop
        )

        rospy.on_shutdown(self.shutdown)

        self.state_pub.publish(
            String(data=self.state)
        )

        rospy.loginfo(
            "Full-scan camera navigation arbiter ready."
        )
        rospy.loginfo(
            "Turning speed: %.2f rad/s",
            self.turning_speed
        )
        rospy.loginfo(
            "Minimum sector clearance: %.2f m",
            self.minimum_sector_clearance
        )

    def navigation_callback(self, message):
        self.latest_navigation_command = message
        self.last_navigation_time = time.time()

    def camera_callback(self, message):
        self.last_camera_time = time.time()

        if message.data:
            self.blocked_count = min(
                self.blocked_count + 1,
                100
            )
            self.free_count = 0
        else:
            self.free_count = min(
                self.free_count + 1,
                100
            )
            self.blocked_count = 0

    def scan_callback(self, message):
        self.latest_scan = message

    def status_callback(self, message):
        self.goal_active = any(
            status.status == 1
            for status in message.status_list
        )

    def change_state(self, new_state):
        if new_state == self.state:
            return

        rospy.logwarn(
            "Arbiter state: %s -> %s",
            self.state,
            new_state
        )

        self.state = new_state

        self.state_pub.publish(
            String(data=self.state)
        )

    def stop_robot(self):
        self.velocity_pub.publish(Twist())

    def valid_scan_distance(self, distance):
        if math.isnan(distance):
            return None

        # An infinite reading normally means no obstacle
        # was detected within the LiDAR range.
        if math.isinf(distance):
            if distance > 0:
                return self.latest_scan.range_max
            return None

        if distance < self.latest_scan.range_min:
            return None

        if distance > self.latest_scan.range_max:
            return self.latest_scan.range_max

        return float(distance)

    def sector_clearance(
        self,
        centre_degrees,
        half_width_degrees=15.0
    ):
        if self.latest_scan is None:
            return 0.0

        centre = math.radians(centre_degrees)
        half_width = math.radians(half_width_degrees)

        sector_ranges = []
        angle = self.latest_scan.angle_min

        for raw_distance in self.latest_scan.ranges:
            angle_difference = math.atan2(
                math.sin(angle - centre),
                math.cos(angle - centre)
            )

            if abs(angle_difference) <= half_width:
                distance = self.valid_scan_distance(
                    raw_distance
                )

                if distance is not None:
                    sector_ranges.append(distance)

            angle += self.latest_scan.angle_increment

        if not sector_ranges:
            return 0.0

        sector_ranges.sort()

        # Lower quartile prevents one distant measurement
        # from making a narrow gap appear safe.
        index = int(
            0.25 * (len(sector_ranges) - 1)
        )

        return float(sector_ranges[index])

    def rotation_clearance(self):
        if self.latest_scan is None:
            return 0.0

        valid_ranges = []

        for raw_distance in self.latest_scan.ranges:
            distance = self.valid_scan_distance(
                raw_distance
            )

            if distance is not None:
                valid_ranges.append(distance)

        if not valid_ranges:
            return 0.0

        valid_ranges.sort()

        # Use a low percentile to reject isolated LiDAR noise.
        index = int(
            0.02 * (len(valid_ranges) - 1)
        )

        return float(valid_ranges[index])

    def choose_best_direction(self):
        # Zero degrees is intentionally excluded because
        # the camera has already classified the front as blocked.
        sector_angles = [
            -180,
            -150,
            -120,
            -90,
            -60,
            -30,
            30,
            60,
            90,
            120,
            150
        ]

        candidates = []
        sector_report = []

        for angle in sector_angles:
            clearance = self.sector_clearance(angle)

            sector_report.append(
                "{}:{:.2f}".format(angle, clearance)
            )

            if clearance >= self.minimum_sector_clearance:
                candidates.append(
                    (clearance, angle)
                )

        rospy.loginfo(
            "Full LiDAR scan [angle:metres]: %s",
            " ".join(sector_report)
        )

        if not candidates:
            rospy.logerr(
                "No LiDAR sector is safe for recovery."
            )
            self.turn_direction = 0
            return False

        # Prefer the greatest clearance.
        # If equal, prefer the smaller rotation.
        candidates.sort(
            key=lambda item: (
                item[0],
                -abs(item[1])
            ),
            reverse=True
        )

        best_clearance, best_angle = candidates[0]

        self.target_angle = float(best_angle)

        # For a directly rearward target, compare the
        # two possible rotation paths.
        if abs(best_angle) == 180:
            left_path = min(
                self.sector_clearance(60),
                self.sector_clearance(120)
            )

            right_path = min(
                self.sector_clearance(-60),
                self.sector_clearance(-120)
            )

            if left_path >= right_path:
                self.turn_direction = 1
            else:
                self.turn_direction = -1

        elif best_angle > 0:
            self.turn_direction = 1

        else:
            self.turn_direction = -1

        direction_name = (
            "LEFT"
            if self.turn_direction > 0
            else "RIGHT"
        )

        rospy.logwarn(
            "Best free sector: %.0f degrees, %.2f m",
            self.target_angle,
            best_clearance
        )

        rospy.logwarn(
            "Continuous recovery direction: %s",
            direction_name
        )

        self.target_angle_pub.publish(
            Float32(data=self.target_angle)
        )

        return True

    def begin_recovery(self):
        self.stop_robot()

        if self.choose_best_direction():
            self.change_state(self.ROTATING)
        else:
            self.change_state(self.SAFETY_STOP)

    def finish_recovery(self):
        rospy.loginfo(
            "Camera view is reliably free. "
            "Returning control to move_base."
        )

        self.stop_robot()

        self.blocked_count = 0
        self.free_count = 0
        self.turn_direction = 0
        self.target_angle = 0.0

        self.target_angle_pub.publish(
            Float32(data=0.0)
        )

        self.change_state(self.NAVIGATION)

    def camera_is_recent(self):
        return (
            time.time() - self.last_camera_time
            <= self.camera_timeout
        )

    def navigation_is_recent(self):
        return (
            time.time() - self.last_navigation_time
            <= self.navigation_timeout
        )

    def control_navigation(self):
        if self.blocked_count >= self.blocked_frames_required:
            rospy.logwarn(
                "Persistent camera blockage detected."
            )
            self.begin_recovery()
            return

        if not self.navigation_is_recent():
            self.stop_robot()
            return

        self.velocity_pub.publish(
            self.latest_navigation_command
        )

    def control_rotation(self):
        if self.free_count >= self.free_frames_required:
            self.finish_recovery()
            return

        clearance = self.rotation_clearance()

        if clearance < self.critical_rotation_clearance:
            rospy.logerr(
                "Obstacle too close for safe rotation: %.2f m",
                clearance
            )

            self.change_state(self.SAFETY_STOP)
            self.stop_robot()
            self.last_retry_time = time.time()
            return

        command = Twist()
        command.linear.x = 0.0
        command.angular.z = (
            self.turn_direction * self.turning_speed
        )

        self.velocity_pub.publish(command)

    def control_safety_stop(self):
        self.stop_robot()

        clearance = self.rotation_clearance()

        if (
            self.free_count >= self.free_frames_required
            and clearance >= self.critical_rotation_clearance
        ):
            self.finish_recovery()
            return

        # Retry full LiDAR analysis once per second.
        if time.time() - self.last_retry_time >= 1.0:
            self.last_retry_time = time.time()

            if (
                clearance
                >= self.critical_rotation_clearance
            ):
                self.begin_recovery()

    def control_loop(self, event):
        if not self.goal_active:
            self.stop_robot()

            if self.state != self.NAVIGATION:
                self.change_state(self.NAVIGATION)

            self.blocked_count = 0
            self.free_count = 0
            self.turn_direction = 0
            return

        if self.latest_scan is None:
            rospy.logwarn_throttle(
                2.0,
                "Waiting for LiDAR scan."
            )
            self.stop_robot()
            return

        if not self.camera_is_recent():
            rospy.logwarn_throttle(
                2.0,
                "Camera safety data is missing or stale."
            )
            self.stop_robot()
            return

        if self.state == self.NAVIGATION:
            self.control_navigation()

        elif self.state == self.ROTATING:
            self.control_rotation()

        elif self.state == self.SAFETY_STOP:
            self.control_safety_stop()

    def shutdown(self):
        self.stop_robot()
        rospy.loginfo(
            "Camera navigation arbiter stopped."
        )


if __name__ == "__main__":
    try:
        CameraNavigationArbiter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
