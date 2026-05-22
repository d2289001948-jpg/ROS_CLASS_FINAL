#pragma once

#include <ros/ros.h>
#include <nav_core/base_local_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/buffer.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vector>
#include <string>
#include <utility>
#include <cmath>

namespace pid_local_planner {

/**
 * PID 局部规划器（运动学模型版）
 *
 * 控制原理：
 *   - 前瞻点：circle-segment 交点，L = clamp(v * lookahead_time, min, max)
 *   - 航向融合：theta_d = (1-k_theta)*切线方向 + k_theta*直连前瞻点方向
 *   - 速度指令：u = R^{-1}(theta) * k * e_xy （运动学逆模型）
 *   - 加速度限制：对 v 和 w 分别做增量限幅（基于里程计反馈）
 *
 * 反馈来源：
 *   /odom          → 当前速度（加速度限制 + 前瞻距离）
 *   costmap TF     → map 系位姿（位置误差计算）
 */
class PIDLocalPlanner : public nav_core::BaseLocalPlanner {
public:
    PIDLocalPlanner();
    ~PIDLocalPlanner();

    void initialize(std::string name,
                    tf2_ros::Buffer* tf,
                    costmap_2d::Costmap2DROS* costmap_ros) override;

    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) override;

    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel) override;

    bool isGoalReached() override;

private:
    // ── 里程计回调 ────────────────────────────────────────────────────────────
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

    // ── 路径剪枝：删除已走过的路径点，返回剩余路径 ───────────────────────────
    std::vector<geometry_msgs::PoseStamped>
        prune(const geometry_msgs::PoseStamped& robot_pose);

    // ── 前瞻点计算（circle-segment 精确交点）────────────────────────────────
    void getLookAheadPoint(double L,
                           const geometry_msgs::PoseStamped& robot_pose,
                           const std::vector<geometry_msgs::PoseStamped>& plan,
                           geometry_msgs::PoseStamped& lookahead,
                           double& theta_trj,
                           double& kappa);

    // ── 运动学逆模型 PID 核心 ────────────────────────────────────────────────
    //   输入：当前状态 (rx,ry,rtheta)、期望状态 (tx,ty,ttheta)
    //   输出：(v_cmd, w_cmd)
    std::pair<double, double> pidControl(double rx, double ry, double rtheta,
                                          double tx, double ty);

    // ── 速度平滑（加速度限制，基于里程计反馈）───────────────────────────────
    double linearRegularization(double v_desired);
    double angularRegularization(double w_desired);

    // ── 几何工具 ──────────────────────────────────────────────────────────────
    double normalizeAngle(double a);
    double poseDist(const geometry_msgs::PoseStamped& a,
                    const geometry_msgs::PoseStamped& b);

    // circle-segment 交点（在机器人坐标系下，圆心在原点）
    std::pair<double, double>
        circleSegmentIntersection(std::pair<double,double> p1,
                                  std::pair<double,double> p2,
                                  double r);

    // ── ROS 句柄 ──────────────────────────────────────────────────────────────
    ros::NodeHandle      private_nh_;
    ros::Subscriber      odom_sub_;
    tf2_ros::Buffer*     tf_;
    costmap_2d::Costmap2DROS* costmap_ros_;

    // ── 路径状态 ──────────────────────────────────────────────────────────────
    std::vector<geometry_msgs::PoseStamped> global_plan_;
    double goal_x_, goal_y_, goal_theta_;

    // ── 里程计缓存 ────────────────────────────────────────────────────────────
    nav_msgs::Odometry base_odom_;

    // ── 控制参数 ──────────────────────────────────────────────────────────────
    // 前瞻
    double lookahead_time_;
    double min_lookahead_dist_;
    double max_lookahead_dist_;

    // 运动学增益
    double k_;        // 位置误差比例系数
    double l_;        // 轮距（差速模型用）
    double k_theta_;  // 航向融合系数 0=切线 1=直连
    double k_rot_;    // 终点对准旋转比例增益（越大越快但越易超调）

    // 速度限制
    double max_v_, min_v_, max_v_inc_;
    double max_w_, min_w_, max_w_inc_;

    // 到达判定
    double xy_goal_tolerance_;
    double yaw_goal_tolerance_;
    double rotate_tol_;      // 旋转调整启动阈值

    // 底盘类型
    bool is_holonomic_;

    // 转弯降速参数
    double min_turn_vel_ratio_;

    // 控制周期
    double d_t_;

    // ── 状态标志 ──────────────────────────────────────────────────────────────
    bool initialized_;
    bool goal_reached_;
};

}  // namespace pid_local_planner
