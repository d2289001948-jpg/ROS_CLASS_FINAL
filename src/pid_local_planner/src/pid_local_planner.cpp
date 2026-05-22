#include "pid_local_planner/pid_local_planner.h"

#include <pluginlib/class_list_macros.h>
#include <costmap_2d/costmap_2d.h>
#include <algorithm>
#include <limits>

PLUGINLIB_EXPORT_CLASS(pid_local_planner::PIDLocalPlanner,
                       nav_core::BaseLocalPlanner)

namespace pid_local_planner {

// ─────────────────────────────────────────────────────────────────────────────
//  构造 / 析构
// ─────────────────────────────────────────────────────────────────────────────
PIDLocalPlanner::PIDLocalPlanner()
    : tf_(nullptr), costmap_ros_(nullptr),
      goal_x_(0), goal_y_(0), goal_theta_(0),
      initialized_(false), goal_reached_(false)
{}

PIDLocalPlanner::~PIDLocalPlanner() {}

// ─────────────────────────────────────────────────────────────────────────────
//  初始化：从参数服务器加载参数，订阅里程计
// ─────────────────────────────────────────────────────────────────────────────
void PIDLocalPlanner::initialize(std::string name,
                                  tf2_ros::Buffer* tf,
                                  costmap_2d::Costmap2DROS* costmap_ros)
{
    if (initialized_) {
        ROS_WARN("PIDLocalPlanner: already initialized.");
        return;
    }

    tf_          = tf;
    costmap_ros_ = costmap_ros;
    private_nh_  = ros::NodeHandle("~/" + name);

    // 前瞻参数
    private_nh_.param("lookahead_time",      lookahead_time_,      0.5);
    private_nh_.param("min_lookahead_dist",  min_lookahead_dist_,  0.3);
    private_nh_.param("max_lookahead_dist",  max_lookahead_dist_,  0.9);

    // 运动学增益
    private_nh_.param("k",       k_,       1.0);
    private_nh_.param("l",       l_,       0.2);
    private_nh_.param("k_theta", k_theta_, 0.5);
    private_nh_.param("k_rot",   k_rot_,   2.0); // 终点旋转比例增益，默认2.0

    // 速度限制
    private_nh_.param("max_v",      max_v_,      0.5);
    private_nh_.param("min_v",      min_v_,      0.0);
    private_nh_.param("max_v_inc",  max_v_inc_,  0.5);
    private_nh_.param("max_w",      max_w_,      1.57);
    private_nh_.param("min_w",      min_w_,      0.0);
    private_nh_.param("max_w_inc",  max_w_inc_,  1.57);

    // 到达判定
    private_nh_.param("xy_goal_tolerance",  xy_goal_tolerance_,  0.2);
    private_nh_.param("yaw_goal_tolerance", yaw_goal_tolerance_,  0.2);
    private_nh_.param("rotate_tolerance",   rotate_tol_,          0.5);

    // 底盘类型
    private_nh_.param("is_holonomic", is_holonomic_, false);

    // 转弯降速参数：转弯时线速度最小保留比例（0.0~1.0，0=完全停止，0.2=保留20%）
    private_nh_.param("min_turn_vel_ratio", min_turn_vel_ratio_, 0.25);

    // 控制周期（与 move_base/controller_frequency 一致）
    double freq;
    ros::NodeHandle nh;
    nh.param("/move_base/controller_frequency", freq, 10.0);
    d_t_ = 1.0 / freq;

    // 订阅里程计（用于加速度限制）
    std::string odom_topic;
    private_nh_.param("odom_topic", odom_topic, std::string("/odom_combined"));
    odom_sub_ = nh.subscribe(odom_topic, 10,
                              &PIDLocalPlanner::odomCallback, this);
    ROS_INFO("PIDLocalPlanner: subscribing to odom topic: %s", odom_topic.c_str());

    initialized_ = true;
    ROS_INFO("PIDLocalPlanner initialized. max_v=%.2f, l=%.3f, k_theta=%.2f",
             max_v_, l_, k_theta_);
}

// ─────────────────────────────────────────────────────────────────────────────
//  接收全局路径
// ─────────────────────────────────────────────────────────────────────────────
bool PIDLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
{
    if (!initialized_) {
        ROS_ERROR("PIDLocalPlanner: not initialized.");
        return false;
    }
    global_plan_ = plan;

    // 检测是否是新目标
    double nx = global_plan_.back().pose.position.x;
    double ny = global_plan_.back().pose.position.y;
    if (nx != goal_x_ || ny != goal_y_) {
        goal_x_       = nx;
        goal_y_       = ny;
        goal_theta_   = tf2::getYaw(global_plan_.back().pose.orientation);
        goal_reached_ = false;
        ROS_INFO("PIDLocalPlanner: new goal (%.3f, %.3f, %.3f)",
                 goal_x_, goal_y_, goal_theta_);
    }
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
//  目标到达判断
// ─────────────────────────────────────────────────────────────────────────────
bool PIDLocalPlanner::isGoalReached()
{
    if (!initialized_) {
        ROS_ERROR("PIDLocalPlanner: not initialized.");
        return false;
    }
    if (goal_reached_) {
        ROS_INFO("PIDLocalPlanner: goal reached!");
        return true;
    }
    return false;
}

// ─────────────────────────────────────────────────────────────────────────────
//  核心控制：computeVelocityCommands
//
//  流程（与 pid_controller.cpp 一致）：
//    1. 获取 map 系机器人位姿（costmap TF）
//    2. 路径剪枝
//    3. 动态前瞻距离 L = clamp(|v|*lookahead_time, min, max)
//    4. 圆弧-线段交点法求前瞻点
//    5. 融合航向角 theta_d
//    6a. 位置已到达 → 原地旋转对准目标偏航
//    6b. 位置未到达 → 运动学逆模型 PID 计算 v, w
//    7. 加速度限制
// ─────────────────────────────────────────────────────────────────────────────
bool PIDLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
    if (!initialized_) {
        ROS_ERROR("PIDLocalPlanner: not initialized.");
        return false;
    }
    if (global_plan_.empty()) {
        cmd_vel.linear.x = cmd_vel.angular.z = 0;
        return true;
    }

    // ── 1. 获取机器人在 map 系下的位姿（直接查 TF，避免 local_costmap 的 odom 坐标系）
    geometry_msgs::TransformStamped ts;
    try {
        ts = tf_->lookupTransform("map", "base_footprint", ros::Time(0));
    } catch (tf2::TransformException& ex) {
        ROS_ERROR("PIDLocalPlanner: cannot get map->base_footprint TF: %s", ex.what());
        return false;
    }

    geometry_msgs::PoseStamped robot_pose;
    robot_pose.header.frame_id    = "map";
    robot_pose.header.stamp       = ts.header.stamp;
    robot_pose.pose.position.x    = ts.transform.translation.x;
    robot_pose.pose.position.y    = ts.transform.translation.y;
    robot_pose.pose.position.z    = ts.transform.translation.z;
    robot_pose.pose.orientation   = ts.transform.rotation;

    double rx     = robot_pose.pose.position.x;
    double ry     = robot_pose.pose.position.y;
    double rtheta = tf2::getYaw(robot_pose.pose.orientation);

    // ── 2. 路径剪枝 ────────────────────────────────────────────────────────
    std::vector<geometry_msgs::PoseStamped> prune_plan = prune(robot_pose);

    // ── 3. 动态前瞻距离 ────────────────────────────────────────────────────
    double vt = std::hypot(base_odom_.twist.twist.linear.x,
                           base_odom_.twist.twist.linear.y);
    double wt = base_odom_.twist.twist.angular.z;

    double L = std::fabs(vt) * lookahead_time_;
    L = std::max(min_lookahead_dist_, std::min(max_lookahead_dist_, L));

    // ── 4. 前瞻点 ──────────────────────────────────────────────────────────
    geometry_msgs::PoseStamped lookahead;
    double theta_trj, kappa;
    getLookAheadPoint(L, robot_pose, prune_plan, lookahead, theta_trj, kappa);

    double lx = lookahead.pose.position.x;
    double ly = lookahead.pose.position.y;

    // ── 5. 融合航向角：切线方向 + 直连前瞻点方向 ──────────────────────────
    double theta_dir = std::atan2(ly - ry, lx - rx);
    double theta_d   = normalizeAngle(
        (1.0 - k_theta_) * theta_trj
        + k_theta_ * normalizeAngle(theta_dir));   // 注意先归一化再混合

    // ── 6a. 位置已到达：仅旋转对准终点偏航 ───────────────────────────────
    double dist_to_goal = std::hypot(goal_x_ - rx, goal_y_ - ry);
    if (dist_to_goal < xy_goal_tolerance_) {
        double e_theta = normalizeAngle(goal_theta_ - rtheta);
        if (std::fabs(e_theta) < yaw_goal_tolerance_) {
            cmd_vel.linear.x  = 0;
            cmd_vel.angular.z = 0;
            goal_reached_     = true;
        } else {
            cmd_vel.linear.x  = 0;
            // P 控制：w = k_rot * e_theta，误差越小速度越慢，自然减速避免超调
            // 原来的 e_theta/d_t_ 是死拍控制（每周期想清零所有误差），
            // 机器人到达容差边界时仍在高速旋转，惯性导致超调。
            cmd_vel.angular.z = angularRegularization(k_rot_ * e_theta);
        }
        return true;
    }

    // ── 6b. 运动学逆模型 PID ───────────────────────────────────────────────
    auto [v_des, w_des] = pidControl(rx, ry, rtheta, lx, ly);

    // ── 7a. 转弯降速：角速度越大，线速度越小 ───────────────────────────────
    //   attenuation = clamp(1 - |w_des|/max_w, min_ratio, 1.0)
    double attenuation = 1.0 - std::fabs(w_des) / max_w_;
    attenuation = std::max(min_turn_vel_ratio_, std::min(1.0, attenuation));
    v_des *= attenuation;

    // ── 7b. 加速度限制 ─────────────────────────────────────────────────────
    cmd_vel.linear.x  = linearRegularization(v_des);
    cmd_vel.linear.y  = 0.0;
    cmd_vel.angular.z = angularRegularization(w_des);

    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
//  运动学逆模型 PID（来自 pid_controller.cpp::_pidControl）
//
//  e     = target_xy - robot_xy
//  sx_dot = k * e
//  R_inv  = [[ cos(theta),  sin(theta)],
//             [-sin(theta)/l, cos(theta)/l]]
//  [v, w] = R_inv * sx_dot
// ─────────────────────────────────────────────────────────────────────────────
std::pair<double, double>
PIDLocalPlanner::pidControl(double rx, double ry, double rtheta,
                              double tx, double ty)
{
    double ex = (tx - rx) * k_;
    double ey = (ty - ry) * k_;

    double v = std::cos(rtheta) * ex + std::sin(rtheta) * ey;
    double w = -std::sin(rtheta) / l_ * ex + std::cos(rtheta) / l_ * ey;

    return {v, w};
}

// ─────────────────────────────────────────────────────────────────────────────
//  加速度限制（来自 controller.cpp::linearRegularization / angularRegularization）
// ─────────────────────────────────────────────────────────────────────────────
double PIDLocalPlanner::linearRegularization(double v_des)
{
    double v     = std::hypot(base_odom_.twist.twist.linear.x,
                              base_odom_.twist.twist.linear.y);
    double v_inc = v_des - v;
    if (std::fabs(v_inc) > max_v_inc_)
        v_inc = std::copysign(max_v_inc_, v_inc);

    double v_cmd = v + v_inc;
    v_cmd = std::max(min_v_, std::min(max_v_, std::fabs(v_cmd)));
    return (v_des >= 0) ? v_cmd : -v_cmd;
}

double PIDLocalPlanner::angularRegularization(double w_des)
{
    w_des = std::max(-max_w_, std::min(max_w_, w_des));

    double w     = base_odom_.twist.twist.angular.z;
    double w_inc = w_des - w;
    if (std::fabs(w_inc) > max_w_inc_)
        w_inc = std::copysign(max_w_inc_, w_inc);

    double w_cmd = w + w_inc;
    return std::max(-max_w_, std::min(max_w_, w_cmd));
}

// ─────────────────────────────────────────────────────────────────────────────
//  路径剪枝（来自 controller.cpp::prune）
//  删除机器人已经经过的路径点，返回剩余路径
// ─────────────────────────────────────────────────────────────────────────────
std::vector<geometry_msgs::PoseStamped>
PIDLocalPlanner::prune(const geometry_msgs::PoseStamped& robot_pose)
{
    if (global_plan_.empty())
        return {};

    // 找搜索上界：costmap 宽度一半对应的路径积分距离内
    double bound = costmap_ros_->getCostmap()->getSizeInMetersX() / 2.0;
    double d_accum = 0.0;
    auto upper = global_plan_.begin();
    for (auto it = global_plan_.begin(); it != global_plan_.end() - 1; ++it) {
        d_accum += poseDist(*it, *(it + 1));
        if (d_accum > bound) { upper = it + 1; break; }
        upper = it + 1;
    }

    // 在上界内找最近路径点
    auto closest = global_plan_.begin();
    double min_d  = std::numeric_limits<double>::max();
    for (auto it = global_plan_.begin(); it != upper; ++it) {
        double d = poseDist(robot_pose, *it);
        if (d < min_d) { min_d = d; closest = it; }
    }

    // 剩余路径
    std::vector<geometry_msgs::PoseStamped> pruned(closest, global_plan_.end());

    // 从全局路径中删除已走过部分
    global_plan_.erase(global_plan_.begin(), closest);

    return pruned;
}

// ─────────────────────────────────────────────────────────────────────────────
//  前瞻点计算（来自 controller.cpp::getLookAheadPoint）
//  使用 circle-segment 交点法精确定位前瞻点
// ─────────────────────────────────────────────────────────────────────────────
void PIDLocalPlanner::getLookAheadPoint(
    double L,
    const geometry_msgs::PoseStamped& robot_pose,
    const std::vector<geometry_msgs::PoseStamped>& plan,
    geometry_msgs::PoseStamped& lookahead,
    double& theta_trj,
    double& kappa)
{
    double rx = robot_pose.pose.position.x;
    double ry = robot_pose.pose.position.y;

    // 找第一个距离 >= L 的路径点
    auto goal_it = std::find_if(plan.begin(), plan.end(),
        [&](const geometry_msgs::PoseStamped& ps) {
            return std::hypot(ps.pose.position.x - rx,
                              ps.pose.position.y - ry) >= L;
        });

    if (goal_it == plan.end()) {
        // 路径剩余不足，直接用终点
        goal_it = std::prev(plan.end());
        lookahead = *goal_it;
        kappa    = 0.0;
        theta_trj = std::atan2(goal_it->pose.position.y - ry,
                               goal_it->pose.position.x - rx);
        return;
    }

    // circle-segment 精确交点
    double gx = goal_it->pose.position.x;
    double gy = goal_it->pose.position.y;
    double px, py;

    if (goal_it == plan.begin()) {
        px = rx; py = ry;
    } else {
        auto prev = std::prev(goal_it);
        px = prev->pose.position.x;
        py = prev->pose.position.y;
    }

    // 转到机器人坐标系（圆心在原点）
    auto [ix, iy] = circleSegmentIntersection(
        {px - rx, py - ry},
        {gx - rx, gy - ry},
        L);

    lookahead.pose.position.x = ix + rx;
    lookahead.pose.position.y = iy + ry;
    lookahead.header = goal_it->header;

    // 路径切线方向
    theta_trj = std::atan2(gy - py, gx - px);

    // 曲率（三点法）
    auto next_it = std::next(goal_it);
    if (next_it != plan.end()) {
        double ax = px, ay = py;
        double bx = gx, by = gy;
        double cx = next_it->pose.position.x;
        double cy = next_it->pose.position.y;

        double a  = std::hypot(bx - cx, by - cy);
        double b  = std::hypot(cx - ax, cy - ay);
        double c  = std::hypot(ax - bx, ay - by);
        double cosB = (a*a + c*c - b*b) / (2*a*c + 1e-9);
        cosB        = std::max(-1.0, std::min(1.0, cosB));
        double sinB = std::sin(std::acos(cosB));
        double cross = (bx-ax)*(cy-ay) - (by-ay)*(cx-ax);
        kappa = std::copysign(2*sinB / (b + 1e-9), cross);
    } else {
        kappa = 0.0;
    }

    if (std::isnan(kappa)) kappa = 0.0;
}

// ─────────────────────────────────────────────────────────────────────────────
//  circle-segment 交点（来自 controller.cpp::circleSegmentIntersection）
//  在机器人坐标系下（圆心 = 原点），找线段 p1→p2 与半径 r 圆的交点
//  返回更靠近 p2（前方）的那个交点
// ─────────────────────────────────────────────────────────────────────────────
std::pair<double, double>
PIDLocalPlanner::circleSegmentIntersection(
    std::pair<double,double> p1,
    std::pair<double,double> p2,
    double r)
{
    double dx = p2.first  - p1.first;
    double dy = p2.second - p1.second;
    double dr2 = dx*dx + dy*dy;

    double D = p1.first * p2.second - p2.first * p1.second;
    double disc = r*r * dr2 - D*D;
    if (disc < 0) disc = 0;
    double sq = std::sqrt(disc);

    double sign_dy = (dy < 0) ? -1.0 : 1.0;
    double x1 = ( D*dy + sign_dy*dx*sq) / dr2;
    double y1 = (-D*dx + std::fabs(dy)*sq) / dr2;
    double x2 = ( D*dy - sign_dy*dx*sq) / dr2;
    double y2 = (-D*dx - std::fabs(dy)*sq) / dr2;

    // 返回更靠近 p2 的交点
    double d1 = (x1-p2.first)*(x1-p2.first) + (y1-p2.second)*(y1-p2.second);
    double d2 = (x2-p2.first)*(x2-p2.first) + (y2-p2.second)*(y2-p2.second);
    return (d1 < d2) ? std::make_pair(x1, y1) : std::make_pair(x2, y2);
}

// ─────────────────────────────────────────────────────────────────────────────
//  工具函数
// ─────────────────────────────────────────────────────────────────────────────
void PIDLocalPlanner::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    base_odom_ = *msg;
}

double PIDLocalPlanner::normalizeAngle(double a)
{
    while (a >  M_PI) a -= 2*M_PI;
    while (a < -M_PI) a += 2*M_PI;
    return a;
}

double PIDLocalPlanner::poseDist(const geometry_msgs::PoseStamped& a,
                                  const geometry_msgs::PoseStamped& b)
{
    return std::hypot(a.pose.position.x - b.pose.position.x,
                      a.pose.position.y - b.pose.position.y);
}

}  // namespace pid_local_planner
