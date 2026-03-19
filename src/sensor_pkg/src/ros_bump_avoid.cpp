#define _USE_MATH_DEFINES
#include "ros/ros.h"
#include "std_msgs/Int16MultiArray.h"
#include "geometry_msgs/Twist.h"
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <algorithm>

namespace {

inline double normalizeAngle(double a)
{
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

inline double shortestAngularDiff(double from, double to)
{
    return normalizeAngle(to - from);
}

inline double quatToYaw(const geometry_msgs::Quaternion& q)
{
    const double qx = q.x;
    const double qy = q.y;
    const double qz = q.z;
    const double qw = q.w;
    return std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
}

enum class State
{
    FORWARD,
    BACKWARD,
    TURN,
    PAUSE,
    STOP
};

struct Params
{
    double loop_hz = 10.0;
    double forward_speed = 0.1;
    double backward_speed = 0.1;
    double backward_distance = 0.2;
    double turn_angle_rad = M_PI / 3.0;     // 60度
    double yaw_tolerance = 0.15;            // 约 8.6度
    double turn_k = 3.0;                    // 角度误差 -> 角速度
    double turn_max = 1.0;                  // 最大角速度
    double pause_before_turn_s = 0.3;
    double pause_after_turn_s = 0.3;
    int bump_queue = 50;
    int odom_queue = 50;
    double log_throttle_s = 0.5;
};

class BumpAvoidance
{
public:
    explicit BumpAvoidance(ros::NodeHandle& nh)
        : nh_(nh), pnh_("~")
    {
        loadParams();
        vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        bump_sub_ = nh_.subscribe("/robot/bump_sensor", params_.bump_queue, &BumpAvoidance::bumpCallback, this);
        odom_sub_ = nh_.subscribe("/odom", params_.odom_queue, &BumpAvoidance::odomCallback, this);
    }

    double loopHz() const { return params_.loop_hz; }

    void step()
    {
        const ros::Time now = ros::Time::now();
        geometry_msgs::Twist cmd;

        switch (state_)
        {
        case State::FORWARD: {
            if (collision_detected_)
            {
                latched_collision_type_ = collision_type_;
                collision_detected_ = false;

                start_backward_x_ = current_x_;
                start_backward_y_ = current_y_;
                transitionTo(State::BACKWARD, now);

                cmd.linear.x = 0.0;
                cmd.angular.z = 0.0;
                ROS_INFO("检测到碰撞，开始后退。type=%d", latched_collision_type_);
            }
            else
            {
                cmd.linear.x = params_.forward_speed;
                cmd.angular.z = 0.0;
            }
            break;
        }

        case State::BACKWARD: {
            const double dist = std::hypot(current_x_ - start_backward_x_, current_y_ - start_backward_y_);
            ROS_INFO_THROTTLE(params_.log_throttle_s,
                              "后退中：dist=%.3f/%.3f  当前(%.3f,%.3f) 起点(%.3f,%.3f)",
                              dist, params_.backward_distance,
                              current_x_, current_y_, start_backward_x_, start_backward_y_);

            if (dist >= params_.backward_distance)
            {
                start_yaw_ = current_yaw_;

                // 1: 正前/左前 -> 右转；2: 右前 -> 左转
                const double sign = (latched_collision_type_ == 2) ? +1.0 : -1.0;
                target_yaw_ = normalizeAngle(start_yaw_ + sign * params_.turn_angle_rad);

                cmd.linear.x = 0.0;
                cmd.angular.z = 0.0;

                pause_until_ = now + ros::Duration(params_.pause_before_turn_s);
                resume_state_ = State::TURN;
                transitionTo(State::PAUSE, now);

                ROS_INFO("后退完成，准备转向。start_yaw=%.3f target_yaw=%.3f", start_yaw_, target_yaw_);
            }
            else
            {
                cmd.linear.x = -params_.backward_speed;
                cmd.angular.z = 0.0;
            }
            break;
        }

        case State::TURN: {
            const double yaw_diff = shortestAngularDiff(current_yaw_, target_yaw_);
            ROS_INFO_THROTTLE(params_.log_throttle_s,
                              "转向中：current_yaw=%.3f target_yaw=%.3f diff=%.3f",
                              current_yaw_, target_yaw_, yaw_diff);

            if (std::fabs(yaw_diff) <= params_.yaw_tolerance)
            {
                cmd.linear.x = 0.0;
                cmd.angular.z = 0.0;

                pause_until_ = now + ros::Duration(params_.pause_after_turn_s);
                resume_state_ = State::FORWARD;
                transitionTo(State::PAUSE, now);

                ROS_INFO("转向完成，准备继续前进。");
            }
            else
            {
                const double w = std::min(params_.turn_max, std::fabs(yaw_diff) * params_.turn_k);
                cmd.linear.x = 0.0;
                cmd.angular.z = (yaw_diff > 0.0) ? w : -w;
            }
            break;
        }

        case State::PAUSE: {
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;

            if (now >= pause_until_)
            {
                transitionTo(resume_state_, now);
            }
            break;
        }

        case State::STOP:
        default:
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
            break;
        }

        vel_pub_.publish(cmd);
    }

private:
    void loadParams()
    {
        pnh_.param("loop_hz", params_.loop_hz, params_.loop_hz);
        pnh_.param("forward_speed", params_.forward_speed, params_.forward_speed);
        pnh_.param("backward_speed", params_.backward_speed, params_.backward_speed);
        pnh_.param("backward_distance", params_.backward_distance, params_.backward_distance);
        pnh_.param("turn_angle_rad", params_.turn_angle_rad, params_.turn_angle_rad);
        pnh_.param("yaw_tolerance", params_.yaw_tolerance, params_.yaw_tolerance);
        pnh_.param("turn_k", params_.turn_k, params_.turn_k);
        pnh_.param("turn_max", params_.turn_max, params_.turn_max);
        pnh_.param("pause_before_turn_s", params_.pause_before_turn_s, params_.pause_before_turn_s);
        pnh_.param("pause_after_turn_s", params_.pause_after_turn_s, params_.pause_after_turn_s);
        pnh_.param("bump_queue", params_.bump_queue, params_.bump_queue);
        pnh_.param("odom_queue", params_.odom_queue, params_.odom_queue);
        pnh_.param("log_throttle_s", params_.log_throttle_s, params_.log_throttle_s);
    }

    void transitionTo(State s, const ros::Time& now)
    {
        state_ = s;
        state_enter_time_ = now;
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
        current_yaw_ = quatToYaw(msg->pose.pose.orientation);
    }

    void bumpCallback(const std_msgs::Int16MultiArray::ConstPtr& msg)
    {
        if (msg->data.size() < 3)
        {
            ROS_WARN_THROTTLE(1.0, "碰撞传感器数据长度不足(<3)，忽略。");
            return;
        }

        // 后退/转向/暂停阶段忽略 bump 抖动，避免重入与类型被覆盖
        if (state_ != State::FORWARD)
        {
            return;
        }

        const bool left_bump = (msg->data[0] != 0);
        const bool front_bump = (msg->data[1] != 0);
        const bool right_bump = (msg->data[2] != 0);

        if (front_bump || left_bump)
        {
            collision_detected_ = true;
            collision_type_ = 1;
        }
        else if (right_bump)
        {
            collision_detected_ = true;
            collision_type_ = 2;
        }
        else
        {
            collision_detected_ = false;
            collision_type_ = 0;
        }
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Publisher vel_pub_;
    ros::Subscriber bump_sub_;
    ros::Subscriber odom_sub_;

    Params params_;
    State state_ = State::FORWARD;
    State resume_state_ = State::FORWARD;
    ros::Time state_enter_time_;
    ros::Time pause_until_;

    double current_x_ = 0.0;
    double current_y_ = 0.0;
    double current_yaw_ = 0.0;

    double start_backward_x_ = 0.0;
    double start_backward_y_ = 0.0;
    double start_yaw_ = 0.0;
    double target_yaw_ = 0.0;

    bool collision_detected_ = false;
    int collision_type_ = 0;
    int latched_collision_type_ = 0;
};

} // namespace

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bump_obstacle_avoidance");
    ros::NodeHandle nh;

    BumpAvoidance node(nh);
    ros::Rate rate(node.loopHz());
    
    while (ros::ok())
    {
        ros::spinOnce();
        node.step();
        rate.sleep();
    }

    return 0;
}
