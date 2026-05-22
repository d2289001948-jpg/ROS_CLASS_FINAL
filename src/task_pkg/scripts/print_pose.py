#!/usr/bin/env python3
import rospy
import tf2_ros
import math

def main():
    rospy.init_node("print_initial_pose", anonymous=True)
    tf_buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buffer)
    rospy.loginfo("等待 map->base_footprint TF...")
    try:
        tf_buffer.can_transform("map", "base_footprint", rospy.Time(0), rospy.Duration(10.0))
        t = tf_buffer.lookup_transform("map", "base_footprint", rospy.Time(0))
        x = t.transform.translation.x
        y = t.transform.translation.y
        q = t.transform.rotation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        rospy.loginfo("=" * 50)
        rospy.loginfo("当前机器人坐标: x=%.3f  y=%.3f  yaw=%.3f rad (%.1f deg)",
                      x, y, yaw, math.degrees(yaw))
        rospy.loginfo("=" * 50)
    except Exception as e:
        rospy.logwarn("无法获取初始坐标: %s", str(e))

if __name__ == "__main__":
    main()
