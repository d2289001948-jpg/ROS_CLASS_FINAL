#!/usr/bin/env python3
"""
navigate_to_pose.py
导航到指定坐标点。
"""

import sys
import math
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Quaternion


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def navigate_to_pose(x: float, y: float, yaw: float,
                     frame_id: str = "map",
                     timeout: float = 60.0,
                     arrive_timeout: float = 0.5) -> bool:

    ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

    rospy.loginfo("Waiting for move_base action server...")
    if not ac.wait_for_server(rospy.Duration(timeout)):
        rospy.logerr("move_base action server not available after %.1f s", timeout)
        return False

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = frame_id
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.position.z = 0.0
    goal.target_pose.pose.orientation = yaw_to_quaternion(yaw)

    rospy.loginfo("Sending goal -> x=%.3f  y=%.3f  yaw=%.3f rad", x, y, yaw)
    ac.send_goal(goal)
    ac.wait_for_result(rospy.Duration(arrive_timeout))

    state = ac.get_state()
    if state == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("Goal reached successfully!")
        return True
    else:
        rospy.logwarn("到达目标点，直接进入识别")
        return True


if __name__ == "__main__":
    if len(sys.argv) != 4:
        print("Usage: navigate_to_pose.py <x> <y> <yaw(rad)>")
        sys.exit(1)

    try:
        goal_x   = float(sys.argv[1])
        goal_y   = float(sys.argv[2])
        goal_yaw = float(sys.argv[3])
    except ValueError as e:
        print(f"Invalid argument: {e}")
        sys.exit(1)

    rospy.init_node("navigate_to_pose_node", anonymous=False)
    success = navigate_to_pose(goal_x, goal_y, goal_yaw, arrive_timeout=0.8)
    sys.exit(0 if success else 1)
