# ROS_class_final — 室内多房间自主导航与识别系统

基于 ROS 的小车室内多房间自主导航系统。状态机驱动，依次前往两个目标房间进行 AprilTag 视觉识别，最后自动返回起点。核心亮点是**运行时动态切换局部规划器**（PID 精准直行 → TEB 复杂避障）。

## 目录结构

```
ROS_class_final/src/
├── task_pkg/            # 主任务包：状态机 + PID导航 + 启动文件
├── pid_local_planner/   # 自定义 PID 局部规划器插件（move_base 插件）
├── pid_nav_pkg/         # 独立 PID 导航器（跳过 move_base 直接控制 /cmd_vel）
├── nav_pkg/             # TEB 导航封装（move_base action client）
├── voice_pkg/           # AprilTag 视觉识别 + 语音播报反馈
├── imu_spin_pkg/        # IMU 原地旋转控制
└── jie_ware/            # 激光扫描定位 + 雷达滤波 + 代价地图自动清理
```

## 包功能说明

### task_pkg — 主任务包

| 文件 | 功能 |
|------|------|
| `scripts/state_machine.py` | **主状态机**：IDLE → GO_ROOM1(PID) → DETECT_ROOM1 → GO_ROOM2(TEB) → DETECT_ROOM2 → RETURN_HOME → DONE |
| `scripts/pid_navigate.py` | 纯 PID 控制器，跳过 move_base，直接发 `/cmd_vel`。短距离直线冲刺用 |
| `scripts/print_pose.py` | 从 TF 读取 `map→base_footprint` 并打印当前坐标和偏航角 |
| `launch/start_all.launch` | **一键启动**：底盘驱动 + lidar_loc 定位 + 雷达滤波 + 代价地图清理 + TEB 导航栈 + TTS 语音合成 |
| `launch/task.launch` | 启动状态机节点，按 Enter 开始执行任务 |
| `launch/navigation_pid.launch` | 导航栈（lidar_loc + lidar_filter + costmap_cleaner + move_base TEB） |
| `launch/move_base_teb.launch` | 纯 TEB move_base（不使用自定义 lidar_loc 时可单独启动） |

### pid_local_planner — 自定义 PID 局部规划器

move_base 插件，通过 `pluginlib` 注册。用于房间 1 的短程精准直行，参数通过 YAML 文件加载，运行时用 `dynamic_reconfigure` 切换：

- `config/pid_controller_params_fast.yaml` — PID 参数（高速模式，max 0.25 m/s）

### pid_nav_pkg — 独立 PID 导航

不依赖 move_base，直接从 TF 读取位姿、计算 PID 控制量、发 `/cmd_vel`。三阶段导航：对准方向 → 直线逼近 → 原地调整偏航角。

### nav_pkg — TEB 导航封装

对 move_base 的 SimpleActionClient 封装，发送目标点后等待到达。

### voice_pkg — AprilTag 识别 + 语音反馈

| 文件 | 功能 |
|------|------|
| `scripts/tag_voice_function.py` | `detect_tag()`：订阅 `/camera/color/image_raw`，检测 tag36h11 id=1；`play_result()`：播放声音 |
| `scripts/tag_audio_once.py` | 独立运行的单次检测节点，按 Enter 后检测一次 tag1 并语音播报结果 |
| `sounds/targetfound.wav` | 识别到目标的声音 |
| `sounds/targetnotfound.wav` | 未识别到目标的声音 |

### imu_spin_pkg — IMU 旋转

通过 IMU 数据控制小车原地旋转到指定角度（`ros_imu_spin.cpp`）。

### jie_ware — 激光定位与滤波

| 节点 | 功能 |
|------|------|
| `lidar_loc` | 激光扫描匹配定位，发布 `map→odom_combined` TF（替代 AMCL） |
| `lidar_filter_node` | 对原始 `/scan` 去噪，发布 `/scan_filtered` |
| `costmap_cleaner` | 自动清除代价地图中的过期障碍物标记 |

## 创新点

1. **运行时动态切换局部规划器**
   - 房间 1 用 PID（短距直行，速度快精度高）
   - 房间 2 用 TEB（长距离有障碍物，需要智能避障）
   - 通过 `dynamic_reconfigure` 在线热切换，不需要重启 move_base

2. **自定义激光定位替代 AMCL**
   - `lidar_loc` 直接做 scan-to-map 匹配，输出 `map→odom_combined` TF
   - `lidar_filter_node` 对原始激光数据去噪，减少代价地图误检
   - `costmap_cleaner` 自动清理过期障碍物，防止假阳性困住规划器

3. **PID 局部规划器插件**
   - 实现 `nav_core::BaseLocalPlanner` 接口，注册为 move_base 插件
   - 参数可通过 `dynamic_reconfigure` 动态调整
   - 适合已知环境中的短距离精准导航

4. **AprilTag 视觉确认 + 语音播报**
   - 到达每个房间后自动检测 tag36h11 id=1
   - 根据检测结果播放对应音频反馈

5. **备用终点机制**
   - 终点被障碍物阻挡时自动切换到 `backup_home`

## 快速启动

### 1. 机器人端（SSH 登录机器人小电脑）

```bash
# 终端 1：启动全部基础设施（底盘驱动 + 导航栈 + TTS）
source ~/ROS_class_final/devel/setup.bash
roslaunch task_pkg start_all.launch
```

等日志出现 `map_server`、`lidar_loc`、`move_base` 就绪后:

```bash
# 终端 2：启动状态机（按 Enter 开始任务）
source ~/ROS_class_final/devel/setup.bash
roslaunch task_pkg task.launch
```

看到 `[FSM] IDLE | 等待键盘指令，按 Enter 开始...` 后，按 Enter 即可启动完整任务序列。

### 2. 本地 PC 端（查看 Rviz 可视化）

本地 PC 需要和机器人在同一局域网，然后设置 ROS Master 指向机器人：

```bash
# 设置机器人 IP 为 ROS Master
export ROS_MASTER_URI=http://172.20.10.4:11311
export ROS_IP=<本机IP>

# 启动 Rviz
rviz -d /home/bcsh/upros_class_code/src/upros_navigation/rviz/show.rviz
```

> `show.rviz` 配置文件在机器人上。如果本地 PC 没有该文件，先 scp 到本地：
> ```bash
> scp bcsh@172.20.10.4:/home/bcsh/upros_class_code/src/upros_navigation/rviz/show.rviz .
> rviz -d ./show.rviz
> ```

Rviz 中可观察：`/map` 地图、`/scan_filtered` 滤波后激光、`/odom` 里程计、TF 树（map→odom_combined→base_footprint）、全局/局部代价地图、全局/局部规划路径。

### 3. 单独发车（不跑完整任务，仅手动导航）

```bash
source ~/ROS_class_final/devel/setup.bash
roslaunch upros_bringup bringup_w2a.launch       # 底盘驱动
roslaunch task_pkg move_base_teb.launch           # move_base TEB 导航栈
```

然后用 Rviz 的 `2D Nav Goal` 工具点选目标，或命令行发目标：

```bash
rosrun nav_pkg navigate_to_pose.py <x> <y> <yaw>
```

## 状态机流程图

```
                        ┌─────────────┐
                        │    IDLE     │ 按 Enter 开始
                        └──────┬──────┘
                               │ switch_to_pid()
                               ▼
                        ┌─────────────┐
                        │  GO_ROOM1   │ PID 精准直行 → 房间1
                        └──────┬──────┘
                               │ 到达
                               ▼
                        ┌─────────────┐
                        │DETECT_ROOM1 │ AprilTag 识别 + 语音
                        └──────┬──────┘
                               │ switch_to_teb()
                               ▼
                        ┌─────────────┐
                        │  GO_ROOM2   │ TEB 智能避障 → 房间2
                        └──────┬──────┘
                               │ 到达
                               ▼
                        ┌─────────────┐
                        │DETECT_ROOM2 │ AprilTag 识别 + 语音
                        └──────┬──────┘
                               │
                               ▼
                        ┌─────────────┐
                        │ RETURN_HOME │ TEB 返回起点
                        └──────┬──────┘
                               │ (失败则自动切备用终点)
                               ▼
                        ┌─────────────┐
                        │    DONE     │ 任务完成
                        └─────────────┘
```

## 导航点坐标

> 坐标随地图变化更新，用 `print_pose.py` 记录新坐标：

```bash
rosrun task_pkg print_pose.py
```

当前坐标（`state_machine.py` 中 `WAYPOINTS`）：

| 点 | x | y | yaw(rad) |
|----|---|---|----------|
| room1 | 2.570438 | -1.721418 | -1.517822 |
| room2 | 2.591515 | -3.648437 | 1.483078 |
| home | -0.291565 | -2.462430 | 3.105691 |
| backup_home | -0.403761 | -2.276059 | 3.095251 |

## 依赖

- ROS Noetic
- move_base + TEB local planner
- `upros_bringup`（底盘驱动）
- `upros_chat`（语音识别 + TTS）
- `upros_navigation`（地图文件 + costmap/planner 配置）
- `apriltag` Python 库（`pip install apriltag`）
- OpenCV（`cv_bridge`）

## 构建

```bash
cd ~/ROS_class_final
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```
