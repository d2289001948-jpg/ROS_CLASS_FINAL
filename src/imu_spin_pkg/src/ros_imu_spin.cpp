#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>

// 全局变量：记录旋转角度、控制状态
double current_angle = 0.0;    // 当前旋转角度（弧度）
double target_angle = M_PI;    // 目标角度：180°=π弧度
bool is_rotating = true;       // 是否继续旋转
ros::Time last_time;           // 上一次回调的时间戳

// 速度发布器
ros::Publisher vel_pub;

// IMU回调函数：计算旋转角度并控制自旋
void imu_spin_callback(const sensor_msgs::Imu::ConstPtr& imu_msg) {
    if (!is_rotating) return;  // 已达到目标角度，停止计算

    // 获取当前时间和时间差（dt）
    ros::Time current_time = ros::Time::now();
    double dt = (current_time - last_time).toSec();
    last_time = current_time;

    // 过滤无效时间差
    if (dt <= 0.0 || dt > 1.0) return;

    // 获取IMU z轴角速度（机器人绕z轴旋转的角速度）
    double angular_vel_z = imu_msg->angular_velocity.z;

    // 积分计算旋转角度（角速度×时间=角度）
    current_angle += angular_vel_z * dt;

    // 打印当前角度和目标角度
    ROS_INFO("Current rotation angle: %.2f rad (target: %.2f rad)", current_angle, target_angle);

    // 发布速度指令：仅z轴角速度，让机器人自旋
    geometry_msgs::Twist vel_cmd;
    if (fabs(current_angle) < target_angle) {
        // 未达到目标角度：继续以0.5 rad/s的速度自旋
        vel_cmd.angular.z = 0.5;  // 自旋速度（可根据机器人调整，0.5是安全值）
        vel_pub.publish(vel_cmd);
    } else {
        // 达到目标角度：停止旋转
        vel_cmd.angular.z = 0.0;
        vel_pub.publish(vel_cmd);
        is_rotating = false;
        ROS_INFO("Spin completed! Total rotation angle: %.2f rad", current_angle);
    }
}

int main(int argc, char** argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "imu_spin_controller");
    ros::NodeHandle nh;

    // 初始化时间戳
    last_time = ros::Time::now();

    // 创建速度发布器：发布到/cmd_vel话题（机器人运动控制话题）
    vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // 订阅IMU数据
    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("/imu/data", 10, imu_spin_callback);

    // 循环等待回调
    ros::spin();

    return 0;
}
