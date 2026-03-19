#include "ros/ros.h"
#include "sensor_msgs/Range.h"
#include "geometry_msgs/Twist.h"
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <algorithm>

namespace {
constexpr double kDefaultLargeDistanceM = 10.0;

inline double clamp(double v, double lo, double hi) { return std::max(lo, std::min(hi, v)); }

inline double normalizeAngle(double a)
{
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

inline double shortestAngularDistance(double from, double to)
{
    return normalizeAngle(to - from);
}
} // namespace

class TofObstacleAvoidance
{
public:
    explicit TofObstacleAvoidance(ros::NodeHandle& nh)
        : nh_(nh)
    {
        // 参数（可在 launch/param 中覆盖）
        nh_.param("obstacle_threshold_m", obstacle_threshold_m_, 0.3);
        nh_.param("backward_distance_m", backward_distance_m_, 0.2);
        nh_.param("forward_speed_mps", forward_speed_mps_, 0.1);
        nh_.param("backward_speed_mps", backward_speed_mps_, 0.1);
        nh_.param("turn_max_speed_rps", turn_max_speed_rps_, 1.0);
        nh_.param("turn_kp", turn_kp_, 3.0);
        nh_.param("turn_goal_tolerance_rad", turn_goal_tolerance_rad_, 0.15);
        nh_.param("turn_angle_rad", turn_angle_rad_, M_PI / 3.0);
        nh_.param("pause_after_backward_s", pause_after_backward_s_, 0.3);
        nh_.param("pause_after_turn_s", pause_after_turn_s_, 0.3);
        nh_.param("log_throttle_s", log_throttle_s_, 1.0);

        vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

        sub_left_ = nh_.subscribe<sensor_msgs::Range>("/ul/sensor1", 10, &TofObstacleAvoidance::rangeLeftCb, this);
        sub_front_ = nh_.subscribe<sensor_msgs::Range>("/ul/sensor2", 10, &TofObstacleAvoidance::rangeFrontCb, this);
        sub_right_ = nh_.subscribe<sensor_msgs::Range>("/ul/sensor3", 10, &TofObstacleAvoidance::rangeRightCb, this);
        odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("/odom", 100, &TofObstacleAvoidance::odomCb, this);

        vel_msg_ = geometry_msgs::Twist(); // 确保初始化为 0
        left_distance_m_ = kDefaultLargeDistanceM;
        front_distance_m_ = kDefaultLargeDistanceM;
        right_distance_m_ = kDefaultLargeDistanceM;
        state_enter_time_ = ros::Time::now();
    }

    void tick()
    {
        const ros::Time now = ros::Time::now();

        // 默认每次都从“安全停下”开始，避免状态切换时沿用上一帧速度
        vel_msg_.linear.x = 0.0;
        vel_msg_.angular.z = 0.0;

        switch (state_) {
        case State::Forward:
            handleForward(now);
            break;
        case State::Backward:
            handleBackward(now);
            break;
        case State::PauseAfterBackward:
            handlePause(now, pause_after_backward_s_, State::Turn);
            break;
        case State::Turn:
            handleTurn(now);
            break;
        case State::PauseAfterTurn:
            handlePause(now, pause_after_turn_s_, State::Forward);
            break;
        case State::Stop:
            // 保持 0 速度
            break;
        }

        vel_pub_.publish(vel_msg_);
    }

private:
    enum class State { Forward, Backward, PauseAfterBackward, Turn, PauseAfterTurn, Stop };
    enum class ObstacleDir { None = 0, Left = 1, Front = 2, Right = 3 };

    void setState(State s)
    {
        state_ = s;
        state_enter_time_ = ros::Time::now();
    }

    void odomCb(const nav_msgs::Odometry::ConstPtr& msg)
    {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;

        // 四元数转 yaw（弧度）
        const double qx = msg->pose.pose.orientation.x;
        const double qy = msg->pose.pose.orientation.y;
        const double qz = msg->pose.pose.orientation.z;
        const double qw = msg->pose.pose.orientation.w;
        current_yaw_ = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
    }

    void rangeLeftCb(const sensor_msgs::Range::ConstPtr& msg) { left_distance_m_ = msg->range; }
    void rangeFrontCb(const sensor_msgs::Range::ConstPtr& msg) { front_distance_m_ = msg->range; }
    void rangeRightCb(const sensor_msgs::Range::ConstPtr& msg) { right_distance_m_ = msg->range; }

    bool obstacleDetected() const
    {
        return (front_distance_m_ < obstacle_threshold_m_) ||
               (left_distance_m_ < obstacle_threshold_m_) ||
               (right_distance_m_ < obstacle_threshold_m_);
    }

    ObstacleDir classifyObstacleDir() const
    {
        // 按“更危险优先级”选择方向：右 -> 前 -> 左（保持你原来的策略）
        if (right_distance_m_ < obstacle_threshold_m_) return ObstacleDir::Right;
        if (front_distance_m_ < obstacle_threshold_m_) return ObstacleDir::Front;
        if (left_distance_m_ < obstacle_threshold_m_) return ObstacleDir::Left;
        return ObstacleDir::None;
    }

    void handleForward(const ros::Time& /*now*/)
    {
        if (obstacleDetected()) {
            obstacle_dir_ = classifyObstacleDir();
            start_backward_x_ = current_x_;
            start_backward_y_ = current_y_;

            ROS_WARN_THROTTLE(log_throttle_s_,
                              "Obstacle detected: left=%.3f front=%.3f right=%.3f dir=%d",
                              left_distance_m_, front_distance_m_, right_distance_m_, static_cast<int>(obstacle_dir_));

            setState(State::Backward);
            return;
        }

        obstacle_dir_ = ObstacleDir::None;
        vel_msg_.linear.x = forward_speed_mps_;
    }

    void handleBackward(const ros::Time& /*now*/)
    {
        const double dx = current_x_ - start_backward_x_;
        const double dy = current_y_ - start_backward_y_;
        const double dist = std::hypot(dx, dy);

        ROS_INFO_THROTTLE(log_throttle_s_, "Backward dist=%.3f/%.3f", dist, backward_distance_m_);

        if (dist >= backward_distance_m_) {
            start_yaw_ = current_yaw_;

            double delta = -turn_angle_rad_; // 默认右转
            if (obstacle_dir_ == ObstacleDir::Right) delta = +turn_angle_rad_; // 右侧障碍 -> 左转
            else if (obstacle_dir_ == ObstacleDir::Left || obstacle_dir_ == ObstacleDir::Front) delta = -turn_angle_rad_;

            target_yaw_ = normalizeAngle(start_yaw_ + delta);

            ROS_INFO_THROTTLE(log_throttle_s_, "Turn target=%.3f (start=%.3f)", target_yaw_, start_yaw_);

            setState(State::PauseAfterBackward);
            return;
        }

        vel_msg_.linear.x = -std::abs(backward_speed_mps_);
    }

    void handleTurn(const ros::Time& /*now*/)
    {
        const double yaw_err = shortestAngularDistance(current_yaw_, target_yaw_);
        ROS_INFO_THROTTLE(log_throttle_s_, "Turning err=%.3f rad", yaw_err);

        if (std::abs(yaw_err) < turn_goal_tolerance_rad_) {
            setState(State::PauseAfterTurn);
            return;
        }

        const double cmd = clamp(turn_kp_ * yaw_err, -turn_max_speed_rps_, turn_max_speed_rps_);
        vel_msg_.angular.z = cmd;
    }

    void handlePause(const ros::Time& now, double pause_s, State next)
    {
        if ((now - state_enter_time_).toSec() >= pause_s) {
            setState(next);
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher vel_pub_;
    ros::Subscriber sub_left_;
    ros::Subscriber sub_front_;
    ros::Subscriber sub_right_;
    ros::Subscriber odom_sub_;

    geometry_msgs::Twist vel_msg_;

    State state_{State::Forward};
    ros::Time state_enter_time_;

    // 传感器/位姿
    double current_x_{0.0};
    double current_y_{0.0};
    double current_yaw_{0.0};
    double left_distance_m_{kDefaultLargeDistanceM};
    double front_distance_m_{kDefaultLargeDistanceM};
    double right_distance_m_{kDefaultLargeDistanceM};

    // 后退/转向状态
    double start_backward_x_{0.0};
    double start_backward_y_{0.0};
    double start_yaw_{0.0};
    double target_yaw_{0.0};
    ObstacleDir obstacle_dir_{ObstacleDir::None};

    // 参数
    double obstacle_threshold_m_{0.3};
    double backward_distance_m_{0.2};
    double forward_speed_mps_{0.1};
    double backward_speed_mps_{0.1};
    double turn_max_speed_rps_{1.0};
    double turn_kp_{3.0};
    double turn_goal_tolerance_rad_{0.15};
    double turn_angle_rad_{M_PI / 3.0};
    double pause_after_backward_s_{0.3};
    double pause_after_turn_s_{0.3};
    double log_throttle_s_{1.0};
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tof_obstacle_avoidance");
    ros::NodeHandle nh;

    // 设置循环频率：10Hz
    ros::Rate rate(10);

    TofObstacleAvoidance node(nh);
    
    while (ros::ok())
    {
        ros::spinOnce(); // 处理回调函数
        node.tick();     // 执行控制逻辑
        rate.sleep();    // 按频率休眠
    }

    return 0;
}
