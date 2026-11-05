#!/usr/bin/env python

import rospy

from std_msgs.msg import String
from actionlib_msgs.msg import GoalStatus
from actionlib_msgs.msg import GoalStatusArray


class MissionManager(object):

    def __init__(self):
        self.mode = None
        self.current_goal_id = None
        self.current_status = None
        self.first_message = True

        self.mode_pub = rospy.Publisher(
            "/mission/mode",
            String,
            queue_size=1,
            latch=True
        )

        rospy.Subscriber(
            "/move_base/status",
            GoalStatusArray,
            self.status_callback
        )

        self.publish_mode("IDLE")

        rospy.loginfo("Mission manager is ready.")
        rospy.loginfo("Waiting for an RViz navigation goal.")

    def publish_mode(self, new_mode):
        if new_mode == self.mode:
            return

        self.mode = new_mode

        # Store the mode for the Jupyter container
        rospy.set_param(
            "/mission/mode",
            new_mode
        )

        # Publish the mode as a normal ROS topic
        self.mode_pub.publish(
            String(data=new_mode)
        )

        rospy.loginfo(
            "MISSION MODE: %s",
            new_mode
        )

    def latest_status(self, status_list):
        return max(
            status_list,
            key=lambda item: (
                item.goal_id.stamp.secs,
                item.goal_id.stamp.nsecs
            )
        )

    def process_status(self, status):
        if status in [
            GoalStatus.PENDING,
            GoalStatus.ACTIVE,
            GoalStatus.PREEMPTING,
            GoalStatus.RECALLING
        ]:
            self.publish_mode("NAVIGATING")

        elif status == GoalStatus.SUCCEEDED:
            rospy.loginfo(
                "Navigation destination reached."
            )

            self.publish_mode("FOLLOW_READY")

        elif status in [
            GoalStatus.ABORTED,
            GoalStatus.REJECTED,
            GoalStatus.LOST
        ]:
            rospy.logwarn(
                "Navigation failed with status %d.",
                status
            )

            self.publish_mode(
                "NAVIGATION_FAILED"
            )

        elif status in [
            GoalStatus.PREEMPTED,
            GoalStatus.RECALLED
        ]:
            rospy.logwarn(
                "Navigation goal was cancelled."
            )

            self.publish_mode("IDLE")

    def status_callback(self, message):
        if not message.status_list:
            return

        latest = self.latest_status(
            message.status_list
        )

        goal_id = latest.goal_id.id
        goal_status = latest.status

        if not goal_id:
            return

        # Ignore a completed goal that existed
        # before this node started.
        if self.first_message:
            self.first_message = False
            self.current_goal_id = goal_id
            self.current_status = goal_status

            if goal_status in [
                GoalStatus.PENDING,
                GoalStatus.ACTIVE
            ]:
                self.process_status(
                    goal_status
                )

            return

        # A different ID means that RViz sent
        # a new navigation goal.
        if goal_id != self.current_goal_id:
            self.current_goal_id = goal_id
            self.current_status = None

            rospy.loginfo(
                "New navigation goal detected: %s",
                goal_id
            )

        # React only when the status changes.
        if goal_status != self.current_status:
            self.current_status = goal_status

            rospy.loginfo(
                "Navigation status changed to: %d",
                goal_status
            )

            self.process_status(
                goal_status
            )


if __name__ == "__main__":
    rospy.init_node(
        "jetbot_mission_manager"
    )

    manager = MissionManager()

    rospy.spin()
