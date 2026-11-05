#!/usr/bin/env python2
# -*- coding: utf-8 -*-
from __future__ import print_function

import math
import time

import rospy

from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String


class CameraNavigationArbiter(object):

    NORMAL = "NAVIGATION"
    TURNING = "CAMERA_RECOVERY"
    HOLD = "RECOVERY_PAUSE"
    LOCKED = "SAFETY_STOP"

    def __init__(self):
        rospy.init_node("camera_navigation_arbiter")

        self.blocked_frames_required = rospy.get_param(
            "~blocked_frames", 5
        )
        self.free_frames_required = rospy.get_param(
            "~free_frames", 3
        )
        self.turning_speed = rospy.get_param(
            "~turning_speed", 0.60
        )
        self.minimum_turn_clearance = rospy.get_param(
            "~minimum_turn_clearance", 0.45
        )
        self.critical_clearance = rospy.get_param(
            "~critical_clearance", 0.25
        )
        self.maximum_turn_time = rospy.get_param(
            "~maximum_turn_time", 2.0
        )
        self.maximum_recovery_time = rospy.get_param(
            "~maximum_recovery_time", 6.0
        )
        self.recovery_pause = rospy.get_param(
            "~recovery_pause", 0.50
        )
        self.camera_timeout = rospy.get_param(
            "~camera_timeout", 1.0
        )
        self.navigation_timeout = rospy.get_param(
            "~navigation_timeout", 0.50
        )

        self.state = self.NORMAL
        self.turn_direction = 0

        self.blocked_count = 0
        self.free_count = 0

        self.goal_active = False
        self.latest_scan = None
        self.latest_navigation_command = Twist()

        self.last_camera_time = 0.0
        self.last_navigation_time = 0.0
        self.state_start_time = time.time()
        self.recovery_start_time = 0.0

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

        self.state_pub.publish(String(data=self.state))

        rospy.loginfo("Camera-navigation arbiter ready.")
        rospy.loginfo(
            "Blocked frames required: %d",
            self.blocked_frames_required
        )
        rospy.loginfo(
            "Free frames required: %d",
            self.free_frames_required
        )
        rospy.loginfo(
            "Minimum turning clearance: %.2f m",
            self.minimum_turn_clearance
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
        # GoalStatus.ACTIVE = 1
        self.goal_active = any(
            status.status == 1
            for status in message.status_list
        )

    def stop_robot(self):
        self.velocity_pub.publish(Twist())

    def change_state(self, new_state):
        if new_state == self.state:
            return

        rospy.logwarn(
            "Arbiter state: %s -> %s",
            self.state,
            new_state
        )

        self.state = new_state
        self.state_start_time = time.time()

        self.state_pub.publish(
            String(data=self.state)
        )

    def sector_clearance(
        self,
        centre_degrees,
        half_width_degrees=25.0
    ):
        if self.latest_scan is None:
            return 0.0

        centre = math.radians(centre_degrees)
        half_width = math.radians(half_width_degrees)

        valid_ranges = []
        angle = self.latest_scan.angle_min

        for distance in self.latest_scan.ranges:
            angle_difference = math.atan2(
                math.sin(angle - centre),
                math.cos(angle - centre)
            )

            inside_sector = (
                abs(angle_difference) <= half_width
            )

            valid_distance = (
                not math.isnan(distance)
                and not math.isinf(distance)
                and distance >= self.latest_scan.range_min
                and distance <= self.latest_scan.range_max
            )

            if inside_sector and valid_distance:
                valid_ranges.append(distance)

            angle += self.latest_scan.angle_increment

        if not valid_ranges:
            return 0.0

        # Conservative lower-quartile distance.
        valid_ranges.sort()
        index = int(0.25 * (len(valid_ranges) - 1))

        return float(valid_ranges[index])

    def choose_turn_direction(self):
        left_clearance = self.sector_clearance(60.0)
        right_clearance = self.sector_clearance(-60.0)

        rospy.loginfo(
            "LiDAR clearance — left: %.2f m, right: %.2f m",
            left_clearance,
            right_clearance
        )

        if (
            left_clearance < self.minimum_turn_clearance
            and right_clearance < self.minimum_turn_clearance
        ):
            self.turn_direction = 0
            rospy.logwarn(
                "Neither turning direction is sufficiently clear."
            )
            return False

        if left_clearance >= right_clearance:
            self.turn_direction = 1
            rospy.logwarn("Recovery direction: LEFT")
        else:
            self.turn_direction = -1
            rospy.logwarn("Recovery direction: RIGHT")

        return True

    def begin_recovery(self, new_recovery):
        if new_recovery:
            self.recovery_start_time = time.time()

        if self.choose_turn_direction():
            self.change_state(self.TURNING)
        else:
            self.change_state(self.HOLD)

        self.stop_robot()

    def finish_recovery(self):
        rospy.loginfo(
            "Camera path is free. Returning control to move_base."
        )

        self.blocked_count = 0
        self.free_count = 0
        self.turn_direction = 0
        self.recovery_start_time = 0.0

        self.change_state(self.NORMAL)
        self.stop_robot()

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

    def control_normal(self):
        if self.blocked_count >= self.blocked_frames_required:
            rospy.logwarn(
                "Persistent camera blockage detected."
            )
            self.begin_recovery(new_recovery=True)
            return

        if not self.navigation_is_recent():
            self.stop_robot()
            return

        self.velocity_pub.publish(
            self.latest_navigation_command
        )

    def control_turning(self):
        if self.free_count >= self.free_frames_required:
            self.finish_recovery()
            return

        total_recovery_time = (
            time.time() - self.recovery_start_time
        )

        if total_recovery_time >= self.maximum_recovery_time:
            rospy.logerr(
                "Maximum recovery time reached. Safety stop."
            )
            self.change_state(self.LOCKED)
            self.stop_robot()
            return

        turning_time = (
            time.time() - self.state_start_time
        )

        if turning_time >= self.maximum_turn_time:
            rospy.logwarn(
                "Turning interval completed. Pausing."
            )
            self.change_state(self.HOLD)
            self.stop_robot()
            return

        if self.turn_direction > 0:
            selected_clearance = self.sector_clearance(60.0)
            opposite_clearance = self.sector_clearance(-60.0)
        else:
            selected_clearance = self.sector_clearance(-60.0)
            opposite_clearance = self.sector_clearance(60.0)

        if selected_clearance < self.critical_clearance:
            rospy.logwarn(
                "Chosen turning side became unsafe."
            )

            if opposite_clearance >= self.minimum_turn_clearance:
                self.turn_direction *= -1
                rospy.logwarn("Switching turning direction.")
            else:
                self.change_state(self.HOLD)
                self.stop_robot()
                return

        recovery_command = Twist()
        recovery_command.linear.x = 0.0
        recovery_command.angular.z = (
            self.turn_direction * self.turning_speed
        )

        self.velocity_pub.publish(recovery_command)

    def control_hold(self):
        self.stop_robot()

        if self.free_count >= self.free_frames_required:
            self.finish_recovery()
            return

        total_recovery_time = (
            time.time() - self.recovery_start_time
        )

        if total_recovery_time >= self.maximum_recovery_time:
            rospy.logerr(
                "No safe camera recovery found. Safety stop."
            )
            self.change_state(self.LOCKED)
            return

        pause_time = time.time() - self.state_start_time

        if pause_time >= self.recovery_pause:
            self.begin_recovery(new_recovery=False)

    def control_locked(self):
        self.stop_robot()

        if self.free_count >= self.free_frames_required:
            self.finish_recovery()

    def control_loop(self, event):
        if not self.goal_active:
            if self.state != self.NORMAL:
                self.change_state(self.NORMAL)

            self.blocked_count = 0
            self.free_count = 0
            self.recovery_start_time = 0.0
            self.stop_robot()
            return

        if self.latest_scan is None:
            rospy.logwarn_throttle(
                2.0,
                "Waiting for LiDAR /scan."
            )
            self.stop_robot()
            return

        if not self.camera_is_recent():
            rospy.logwarn_throttle(
                2.0,
                "Camera safety data missing or stale."
            )
            self.stop_robot()
            return

        if self.state == self.NORMAL:
            self.control_normal()

        elif self.state == self.TURNING:
            self.control_turning()

        elif self.state == self.HOLD:
            self.control_hold()

        elif self.state == self.LOCKED:
            self.control_locked()

    def shutdown(self):
        self.stop_robot()
        rospy.loginfo(
            "Camera-navigation arbiter stopped."
        )


if __name__ == "__main__":
    try:
        CameraNavigationArbiter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
