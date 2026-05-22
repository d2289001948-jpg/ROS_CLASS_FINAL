#!/usr/bin/env python3
"""
state_machine.py
IDLE -> GO_ROOM1(PID) -> DETECT_ROOM1 -> GO_ROOM2(TEB) -> DETECT_ROOM2 -> RETURN_HOME -> DONE

局部规划器切换方式：dynamic_reconfigure（不重启 move_base，TF/代价地图全程保持）
  启动时：TEB
  IDLE->GO_ROOM1 前：动态换成 PID（rosparam 预加载参数 + reconfigure 换插件）
  DETECT_ROOM1 开始：后台线程动态换回 TEB
  播报结束后 join，GO_ROOM2 直接用 TEB
"""

import sys
import math
import enum
import threading
import subprocess
import rospy
import rospkg
import actionlib
import dynamic_reconfigure.client
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Quaternion

sys.path.insert(0, rospkg.RosPack().get_path('voice_pkg') + '/scripts')
from tag_voice_function import detect_tag, play_result

WAYPOINTS = {
    "room1": {"x":  2.570438, "y": -1.721418, "yaw": -1.517822},  # 房间1坐标
    "room2": {"x":  2.591515, "y": -3.648437, "yaw":  1.483078},  # 房间2坐标
    "home":  {"x": -0.291565, "y": -2.462430, "yaw":  3.105691},  # 主终点
    "backup_home": {"x": -0.403761, "y": -2.276059, "yaw":  3.095251},  # 备用终点
}

class State(enum.Enum):
    IDLE         = "IDLE"
    GO_ROOM1     = "GO_ROOM1"
    DETECT_ROOM1 = "DETECT_ROOM1"
    GO_ROOM2     = "GO_ROOM2"
    DETECT_ROOM2 = "DETECT_ROOM2"
    RETURN_HOME  = "RETURN_HOME"
    DONE         = "DONE"
    ERROR        = "ERROR"

# ── 局部规划器插件名 ──────────────────────────────────────────────────────
PLANNER_TEB = "teb_local_planner/TebLocalPlannerROS"
PLANNER_PID = "pid_local_planner/PIDLocalPlanner"

def _reconfigure_planner(planner_name):
    """通过 dynamic_reconfigure 原地切换 move_base 的局部规划器插件。"""
    try:
        client = dynamic_reconfigure.client.Client("/move_base", timeout=10.0)
        client.update_configuration({"base_local_planner": planner_name})
        rospy.loginfo("[PLANNER] 切换为: %s", planner_name.split("/")[-1])
    except Exception as e:
        rospy.logwarn("[PLANNER] reconfigure 失败: %s", e)

def switch_to_pid():
    """
    切换到 PID：
      1. rosparam 预加载 PID 参数到 /move_base/PIDLocalPlanner/
      2. dynamic_reconfigure 换插件（initialize 时会读上面的参数）
    """
    rospy.loginfo("[PLANNER] 切换到 PID...")
    pid_yaml = rospkg.RosPack().get_path("pid_local_planner") +                "/config/pid_controller_params_fast.yaml"
    # 加载到 /move_base 下（yaml 顶层 key 是 PIDLocalPlanner，会落到 /move_base/PIDLocalPlanner/）
    subprocess.call(["rosparam", "load", pid_yaml, "/move_base"])
    _reconfigure_planner(PLANNER_PID)
    rospy.sleep(0.5)  # 等插件 initialize 完成
    rospy.loginfo("[PLANNER] PID 就绪")

def switch_to_teb():
    """切换回 TEB（参数已在启动时由 launch 加载，直接换插件即可）。"""
    rospy.loginfo("[PLANNER] 切换回 TEB...")
    _reconfigure_planner(PLANNER_TEB)
    rospy.sleep(0.5)
    rospy.loginfo("[PLANNER] TEB 就绪")

# ── 导航工具 ──────────────────────────────────────────────────────────────
def yaw_to_quaternion(yaw):
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q

def navigate_to(name, timeout=90.0, retries=2):
    wp = WAYPOINTS[name]
    x, y, yaw = wp["x"], wp["y"], wp["yaw"]
    ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    if not ac.wait_for_server(rospy.Duration(timeout)):
        rospy.logerr("[NAV] move_base not available!")
        return False
    for attempt in range(retries + 1):
        if attempt > 0:
            rospy.logwarn("[NAV] 第 %d 次重试 -> %s", attempt, name)
            rospy.sleep(1.0)
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation = yaw_to_quaternion(yaw)
        rospy.loginfo("[NAV] -> %s (%.3f, %.3f) [%d/%d]", name, x, y, attempt+1, retries+1)
        ac.send_goal(goal)
        ac.wait_for_result()
        if ac.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("[NAV] Reached: %s", name)
            return True
        rospy.logwarn("[NAV] Failed: %s", ac.get_goal_status_text())
    rospy.logerr("[NAV] 全部重试失败: %s", name)
    return False

# ── 状态机 ────────────────────────────────────────────────────────────────
class TaskStateMachine:
    def __init__(self):
        self.state = State.IDLE

    def _transition(self, next_state):
        rospy.loginfo("[FSM] %s -> %s", self.state.value, next_state.value)
        self.state = next_state

    def _log(self, detail=""):
        rospy.loginfo("[FSM] %s%s", self.state.value,
                      f" | {detail}" if detail else "")

    def run(self):
        rospy.loginfo("[FSM] 任务启动")
        while not rospy.is_shutdown():

            if self.state == State.IDLE:
                rospy.sleep(1.0)
                # 第一段用 PID，在发目标前切换好
                switch_to_pid()
                rospy.loginfo("[FSM] 初始化完成，按 Enter 开始导航...")
                input()
                try:
                    rospy.wait_for_service('/move_base/clear_costmaps', timeout=3.0)
                    rospy.ServiceProxy('/move_base/clear_costmaps', Empty)()
                    rospy.loginfo("[FSM] 代价地图已清除")
                except Exception as e:
                    rospy.logwarn("[FSM] 清除代价地图失败: %s", e)
                self._transition(State.GO_ROOM1)

            elif self.state == State.GO_ROOM1:
                self._log("PID | 导航至房间1")
                ok = navigate_to("room1")
                self._transition(State.DETECT_ROOM1 if ok else State.ERROR)

            elif self.state == State.DETECT_ROOM1:
                self._log("识别房间1 AprilTag")
                found = detect_tag()
                # 切回 TEB（快速，约 0.5s），完成后立即导航
                teb_thread = threading.Thread(target=switch_to_teb, daemon=True)
                teb_thread.start()
                teb_thread.join()
                # 播报在后台运行，不阻塞导航
                audio_thread = threading.Thread(target=play_result, args=(found,), daemon=True)
                audio_thread.start()
                self._transition(State.GO_ROOM2)

            elif self.state == State.GO_ROOM2:
                self._log("TEB | 导航至房间2")
                ok = navigate_to("room2")
                self._transition(State.DETECT_ROOM2 if ok else State.ERROR)

            elif self.state == State.DETECT_ROOM2:
                self._log("识别房间2 AprilTag")
                found = detect_tag()
                # 播报在后台运行，不阻塞返回导航
                audio_thread = threading.Thread(target=play_result, args=(found,), daemon=True)
                audio_thread.start()
                self._transition(State.RETURN_HOME)

            elif self.state == State.RETURN_HOME:
                self._log("TEB | 返回主终点")
                ok = navigate_to("home")
                
                # 主终点失败自动切换备用终点
                if not ok:
                    rospy.logwarn("[FSM] 主终点有障碍，自动切换到备用终点")
                    ok = navigate_to("backup_home")
                
                self._transition(State.DONE if ok else State.ERROR)

            elif self.state == State.DONE:
                rospy.loginfo("[FSM] ========== 任务结束 ==========")
                break

            elif self.state == State.ERROR:
                rospy.logerr("[FSM] 任务出错，停止。")
                sys.exit(1)

if __name__ == "__main__":
    rospy.init_node("task_state_machine", anonymous=False)
    TaskStateMachine().run()