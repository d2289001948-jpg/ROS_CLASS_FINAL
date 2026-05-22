#!/usr/bin/env python3
"""
pid_navigator.py
三段式 P 控制导航（绕过 TEB / move_base），用于路线已知的固定段。

阶段 1  原地对准目标方向
阶段 2  直线前进 + 偏航纠偏（比例控制）
阶段 3  原地调整到目标偏航角

作为库导入：
    from pid_nav_pkg.scripts.pid_navigator import PIDNavigator
    nav = PIDNavigator()
    ok = nav.navigate(x, y, yaw)         # True=到达, False=超时

作为节点运行：
    rosrun pid_nav_pkg pid_navigator.py <x> <y> <yaw>
"""

import sys
import math
import rospy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped


# ── 工具函数 ──────────────────────────────────────────────────────────────────

def _norm(a):
    """角度归一化到 (-π, π]"""
    while a >  math.pi: a -= 2 * math.pi
    while a < -math.pi: a += 2 * math.pi
    return a

def _quat_to_yaw(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


# ── 主类 ──────────────────────────────────────────────────────────────────────

class PIDNavigator:
    """
    可调参数均为类属性，实例化后可直接覆盖：
        nav = PIDNavigator()
        nav.MAX_LINEAR = 0.4
    """

    # 控制增益
    KP_LINEAR   = 0.6    # 线速度比例增益
    KP_ANGULAR  = 2.0    # 角速度比例增益

    # 速度上限
    MAX_LINEAR  = 0.35   # 最大线速度 m/s（TEB 配置为 0.25，此处更快）
    MAX_ANGULAR = 1.5    # 最大角速度 rad/s

    # 到达容差
    XY_TOL      = 0.15   # 位置容差 m
    YAW_TOL     = 0.10   # 偏航容差 rad
    HEADING_TOL = 0.12   # Phase1 对准退出容差 rad

    # 控制频率
    CTRL_RATE   = 20     # Hz

    def __init__(self, cmd_topic='/cmd_vel', pose_topic='/amcl_pose'):
        self._x   = None
        self._y   = None
        self._yaw = None
        self._pub = rospy.Publisher(cmd_topic, Twist, queue_size=1)
        self._sub = rospy.Subscriber(pose_topic, PoseWithCovarianceStamped,
                                     self._pose_cb, queue_size=1)
        rospy.loginfo('[PIDNav] 初始化，等待 %s ...', pose_topic)

    # ── 内部 ──────────────────────────────────────────────────────────────────

    def _pose_cb(self, msg):
        p = msg.pose.pose
        self._x   = p.position.x
        self._y   = p.position.y
        self._yaw = _quat_to_yaw(p.orientation)

    def _wait_pose(self, timeout=6.0):
        t0   = rospy.Time.now()
        rate = rospy.Rate(20)
        while self._x is None:
            if (rospy.Time.now() - t0).to_sec() > timeout:
                rospy.logerr('[PIDNav] 超时：未收到位姿话题')
                return False
            rate.sleep()
        return True

    def _stop(self):
        self._pub.publish(Twist())

    def _clamp(self, v, limit):
        return max(-limit, min(limit, v))

    # ── 公开接口 ──────────────────────────────────────────────────────────────

    def navigate(self, goal_x: float, goal_y: float, goal_yaw: float,
                 timeout: float = 30.0) -> bool:
        """
        PID 导航到 (goal_x, goal_y, goal_yaw)。
        返回 True=到达，False=超时/错误。
        """
        if not self._wait_pose():
            return False

        rate = rospy.Rate(self.CTRL_RATE)
        t0   = rospy.Time.now()
        cmd  = Twist()

        def timed_out():
            return (rospy.Time.now() - t0).to_sec() > timeout

        # ── 阶段 1：原地对准目标方向 ──────────────────────────────────────────
        rospy.loginfo('[PIDNav] Phase1 对准方向 -> (%.3f, %.3f)', goal_x, goal_y)
        while not rospy.is_shutdown():
            if timed_out():
                self._stop()
                rospy.logwarn('[PIDNav] Phase1 超时')
                return False
            dx, dy = goal_x - self._x, goal_y - self._y
            if math.hypot(dx, dy) < self.XY_TOL:
                break
            err = _norm(math.atan2(dy, dx) - self._yaw)
            if abs(err) < self.HEADING_TOL:
                break
            cmd.linear.x  = 0.0
            cmd.angular.z = self._clamp(self.KP_ANGULAR * err, self.MAX_ANGULAR)
            self._pub.publish(cmd)
            rate.sleep()
        self._stop()
        rospy.sleep(0.1)

        # ── 阶段 2：直线前进 + 偏航纠偏 ──────────────────────────────────────
        rospy.loginfo('[PIDNav] Phase2 前进 -> (%.3f, %.3f)', goal_x, goal_y)
        while not rospy.is_shutdown():
            if timed_out():
                self._stop()
                rospy.logwarn('[PIDNav] Phase2 超时')
                return False
            dx, dy = goal_x - self._x, goal_y - self._y
            dist   = math.hypot(dx, dy)
            if dist < self.XY_TOL:
                break
            err = _norm(math.atan2(dy, dx) - self._yaw)
            cmd.linear.x  = min(self.MAX_LINEAR, self.KP_LINEAR * dist)
            cmd.angular.z = self._clamp(self.KP_ANGULAR * err, self.MAX_ANGULAR)
            self._pub.publish(cmd)
            rate.sleep()
        self._stop()
        rospy.sleep(0.1)

        # ── 阶段 3：原地调整终点偏航 ──────────────────────────────────────────
        rospy.loginfo('[PIDNav] Phase3 调整偏航 -> %.3f rad', goal_yaw)
        while not rospy.is_shutdown():
            if timed_out():
                self._stop()
                rospy.logwarn('[PIDNav] Phase3 超时')
                return False
            err = _norm(goal_yaw - self._yaw)
            if abs(err) < self.YAW_TOL:
                break
            cmd.linear.x  = 0.0
            cmd.angular.z = self._clamp(self.KP_ANGULAR * err, self.MAX_ANGULAR)
            self._pub.publish(cmd)
            rate.sleep()
        self._stop()

        rospy.loginfo('[PIDNav] 到达 (%.3f, %.3f, yaw=%.3f)', goal_x, goal_y, goal_yaw)
        return True


# ── 作为独立节点运行 ──────────────────────────────────────────────────────────
if __name__ == '__main__':
    if len(sys.argv) != 4:
        print('Usage: pid_navigator.py <x> <y> <yaw_rad>')
        sys.exit(1)
    try:
        gx, gy, gyaw = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])
    except ValueError as e:
        print(f'Invalid argument: {e}')
        sys.exit(1)

    rospy.init_node('pid_navigator_node', anonymous=False)
    nav = PIDNavigator()
    ok  = nav.navigate(gx, gy, gyaw)
    sys.exit(0 if ok else 1)
