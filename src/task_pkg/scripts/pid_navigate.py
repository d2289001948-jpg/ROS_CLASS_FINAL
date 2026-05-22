#!/usr/bin/env python3
"""
pid_navigate.py
基于 PID 控制直接发布 /cmd_vel 快速导航到目标点。
比 move_base 响应更快，适合短距离直线冲刺。

用法（作为库）:
  from pid_navigate import pid_navigate_to
  success = pid_navigate_to(x, y, yaw)
"""

import math
import rospy
from geometry_msgs.msg import Twist
import tf2_ros


# ─────────────────────────── PID 参数（可调）──────────────────────────────
# 底盘参数：差速两轮，轮距 0.216m，轮径 0.099m
# 硬件上限：max_vel_x=0.25 m/s，max_vel_theta=1.5 rad/s
LINEAR_KP  = 0.8    # 线速度比例增益
LINEAR_KI  = 0.0    # 线速度积分增益
LINEAR_KD  = 0.05   # 线速度微分增益

ANGULAR_KP = 1.5    # 角速度比例增益
ANGULAR_KI = 0.0    # 角速度积分增益
ANGULAR_KD = 0.10   # 角速度微分增益

MAX_LINEAR  = 0.25  # 最大线速度 m/s（匹配底盘硬件上限）
MAX_ANGULAR = 1.2   # 最大角速度 rad/s（低于硬件上限 1.5，留余量）
MIN_LINEAR  = 0.05  # 最小线速度（防止停滞）

DIST_TOLERANCE = 0.15   # 到达位置容差 m
YAW_TOLERANCE  = 0.10   # 到达朝向容差 rad
CONTROL_RATE   = 20     # 控制频率 Hz


class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp, self.ki, self.kd = kp, ki, kd
        self._integral = 0.0
        self._prev_error = 0.0

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0

    def compute(self, error, dt):
        self._integral += error * dt
        derivative = (error - self._prev_error) / dt if dt > 0 else 0.0
        self._prev_error = error
        return self.kp * error + self.ki * self._integral + self.kd * derivative


def get_robot_pose(tf_buffer):
    """从 TF 获取机器人在 map 坐标系下的位姿，返回 (x, y, yaw)。"""
    try:
        trans = tf_buffer.lookup_transform("map", "base_link", rospy.Time(0), rospy.Duration(1.0))
        x = trans.transform.translation.x
        y = trans.transform.translation.y
        q = trans.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        return x, y, yaw
    except Exception as e:
        rospy.logwarn_throttle(2.0, "[PID] TF 获取失败: %s", str(e))
        return None


def normalize_angle(angle):
    """将角度归一化到 [-pi, pi]。"""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def pid_navigate_to(x: float, y: float, yaw: float,
                    timeout: float = 30.0) -> bool:
    """
    用 PID 控制导航到目标点。

    Parameters
    ----------
    x, y    : 目标位置（map 坐标系，米）
    yaw     : 目标朝向（弧度）
    timeout : 最长等待时间（秒）

    Returns
    -------
    True — 成功到达，False — 超时或失败
    """
    cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    linear_pid  = PIDController(LINEAR_KP,  LINEAR_KI,  LINEAR_KD)
    angular_pid = PIDController(ANGULAR_KP, ANGULAR_KI, ANGULAR_KD)

    rate     = rospy.Rate(CONTROL_RATE)
    dt       = 1.0 / CONTROL_RATE
    deadline = rospy.Time.now() + rospy.Duration(timeout)

    rospy.loginfo("[PID] 目标: x=%.3f  y=%.3f  yaw=%.3f", x, y, yaw)
    rospy.sleep(0.5)  # 等 TF 就绪

    phase = "APPROACH"   # APPROACH -> ALIGN

    while not rospy.is_shutdown():
        if rospy.Time.now() > deadline:
            rospy.logwarn("[PID] 超时！")
            _stop(cmd_pub)
            return False

        pose = get_robot_pose(tf_buffer)
        if pose is None:
            rate.sleep()
            continue

        rx, ry, ryaw = pose
        dx = x - rx
        dy = y - ry
        dist = math.sqrt(dx * dx + dy * dy)
        angle_to_goal = math.atan2(dy, dx)
        heading_error = normalize_angle(angle_to_goal - ryaw)

        twist = Twist()

        if phase == "APPROACH":
            if dist < DIST_TOLERANCE:
                rospy.loginfo("[PID] 到达位置，开始对齐朝向")
                phase = "ALIGN"
                linear_pid.reset()
                angular_pid.reset()
                continue

            # 先对准方向再前进
            if abs(heading_error) > 0.5:
                twist.linear.x  = 0.0
                twist.angular.z = _clamp(angular_pid.compute(heading_error, dt), -MAX_ANGULAR, MAX_ANGULAR)
            else:
                linear_out  = _clamp(linear_pid.compute(dist, dt), MIN_LINEAR, MAX_LINEAR)
                angular_out = _clamp(angular_pid.compute(heading_error, dt), -MAX_ANGULAR, MAX_ANGULAR)
                twist.linear.x  = linear_out
                twist.angular.z = angular_out

            rospy.loginfo_throttle(1.0, "[PID] dist=%.2fm  heading_err=%.2frad  v=%.2f  w=%.2f",
                                   dist, heading_error, twist.linear.x, twist.angular.z)

        elif phase == "ALIGN":
            final_yaw_err = normalize_angle(yaw - ryaw)
            if abs(final_yaw_err) < YAW_TOLERANCE:
                _stop(cmd_pub)
                rospy.loginfo("[PID] 到达目标！")
                return True
            twist.angular.z = _clamp(angular_pid.compute(final_yaw_err, dt), -MAX_ANGULAR, MAX_ANGULAR)

        cmd_pub.publish(twist)
        rate.sleep()

    _stop(cmd_pub)
    return False


def _stop(pub):
    pub.publish(Twist())


def _clamp(val, lo, hi):
    return max(lo, min(hi, val))


# ─────────────────────────── 独立运行入口 ───────────────────────────────────
if __name__ == "__main__":
    import sys
    if len(sys.argv) != 4:
        print("Usage: pid_navigate.py <x> <y> <yaw>")
        sys.exit(1)
    rospy.init_node("pid_navigate_node", anonymous=False)
    ok = pid_navigate_to(float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]))
    sys.exit(0 if ok else 1)
