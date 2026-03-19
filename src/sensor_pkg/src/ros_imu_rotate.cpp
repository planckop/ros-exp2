#define _USE_MATH_DEFINES
#include "ros/ros.h"
#include "ros/master.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Twist.h"
#include <cmath>
#include <string>
#include <vector>

namespace {
double normalizeAngle(double a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

double shortestAngularDistance(double from, double to) {
    return normalizeAngle(to - from);
}

std::string pickFirstExistingTopic(const std::vector<std::string>& candidates) {
    ros::master::V_TopicInfo topics;
    ros::master::getTopics(topics);
    for (const auto& name : candidates) {
        for (const auto& t : topics) {
            if (t.name == name) return name;
        }
    }
    return std::string{};
}
} // namespace

class ImuRotator {
public:
    explicit ImuRotator(ros::NodeHandle& nh)
        : nh_(nh), pnh_("~") {
        // 可通过参数覆盖（单位：rad / s）
        pnh_.param("target_delta_yaw", target_delta_yaw_, M_PI);
        pnh_.param("tolerance", tolerance_, 0.02);
        pnh_.param("angular_velocity", angular_velocity_, M_PI / 10.0);
        pnh_.param("max_angular_velocity", max_angular_velocity_, angular_velocity_);
        pnh_.param("min_angular_velocity", min_angular_velocity_, angular_velocity_);
        pnh_.param("control_rate_hz", control_rate_hz_, 10.0);
        pnh_.param("rotate_timeout_sec", rotate_timeout_sec_, 20.0);
        pnh_.param("init_timeout_sec", init_timeout_sec_, 3.0);

        // 速度发布器
        vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

        // IMU 话题：优先用参数，否则自动探测常见名称
        std::string imu_topic_param;
        pnh_.param<std::string>("imu_topic", imu_topic_param, std::string{});
        if (!imu_topic_param.empty()) {
            imu_topic_ = imu_topic_param;
        } else {
            imu_topic_ = pickFirstExistingTopic({"/imu", "/imu/data", "/imu/data_raw"});
        }

        if (imu_topic_.empty()) {
            ROS_ERROR("未找到 IMU 话题（尝试 /imu, /imu/data, /imu/data_raw），也未设置 ~imu_topic 参数。");
            imu_ok_ = false;
            return;
        }

        imu_sub_ = nh_.subscribe(imu_topic_, 100, &ImuRotator::imuCallback, this);
        imu_ok_ = true;
        ROS_INFO_STREAM("订阅 IMU 话题: " << imu_topic_);
    }

    bool ok() const { return imu_ok_; }

    void spin() {
        ros::Rate rate(control_rate_hz_);

        // 等待首次 IMU 数据初始化
        const ros::Time init_deadline = ros::Time::now() + ros::Duration(init_timeout_sec_);
        while (ros::ok() && !is_initialized_) {
            ros::spinOnce();
            if (ros::Time::now() > init_deadline) break;
            rate.sleep();
        }

        if (!is_initialized_) {
            ROS_ERROR("等待 IMU 初始化超时（%.2fs）。", init_timeout_sec_);
            publishStopOnce();
            return;
        }

        is_rotating_ = true;
        rotation_start_time_ = ros::Time::now();

        while (ros::ok() && !rotation_completed_) {
            ros::spinOnce();
            controlStep();
            rate.sleep();
        }

        publishStopOnce();
    }

private:
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
        const double qx = msg->orientation.x;
        const double qy = msg->orientation.y;
        const double qz = msg->orientation.z;
        const double qw = msg->orientation.w;

        // yaw（弧度），范围约 [-pi, pi]
        current_yaw_ = std::atan2(2.0 * (qw * qz + qx * qy),
                                 1.0 - 2.0 * (qy * qy + qz * qz));

        if (!is_initialized_) {
            initial_yaw_ = current_yaw_;
            target_yaw_ = normalizeAngle(initial_yaw_ + target_delta_yaw_);
            is_initialized_ = true;
        }
    }

    void publishStopOnce() {
        if (stop_published_) return;
        geometry_msgs::Twist msg;
        msg.linear.x = 0.0;
        msg.angular.z = 0.0;
        vel_pub_.publish(msg);
        stop_published_ = true;
    }

    void controlStep() {
        if (!is_rotating_ || rotation_completed_) return;

        if ((ros::Time::now() - rotation_start_time_).toSec() > rotate_timeout_sec_) {
            ROS_WARN("旋转超时（%.2fs），强制停止。", rotate_timeout_sec_);
            rotation_completed_ = true;
            is_rotating_ = false;
            return;
        }

        const double yaw_err = shortestAngularDistance(current_yaw_, target_yaw_);
        if (std::fabs(yaw_err) < tolerance_) {
            rotation_completed_ = true;
            is_rotating_ = false;
            return;
        }

        // 简单 P 控制：误差越小速度越小，避免接近目标时来回抖
        const double kP = 1.0;
        double w = kP * yaw_err;

        // 限幅
        const double abs_w = std::fabs(w);
        const double abs_w_clamped = std::max(min_angular_velocity_, std::min(max_angular_velocity_, abs_w));
        w = (w >= 0.0) ? abs_w_clamped : -abs_w_clamped;

        geometry_msgs::Twist cmd;
        cmd.linear.x = 0.0;
        cmd.angular.z = w;
        vel_pub_.publish(cmd);
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Publisher vel_pub_;
    ros::Subscriber imu_sub_;
    std::string imu_topic_;
    bool imu_ok_{false};

    // 参数
    double target_delta_yaw_{M_PI};
    double tolerance_{0.02};
    double angular_velocity_{M_PI / 10.0}; // 默认值（兼容你原来的行为）
    double max_angular_velocity_{M_PI / 10.0};
    double min_angular_velocity_{M_PI / 10.0};
    double control_rate_hz_{10.0};
    double rotate_timeout_sec_{20.0};
    double init_timeout_sec_{3.0};

    // 状态
    double current_yaw_{0.0};
    double initial_yaw_{0.0};
    double target_yaw_{0.0};
    bool is_initialized_{false};
    bool is_rotating_{false};
    bool rotation_completed_{false};
    bool stop_published_{false};
    ros::Time rotation_start_time_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_rotation");
    ros::NodeHandle nh;
    ImuRotator rotator(nh);
    if (!rotator.ok()) return 1;
    rotator.spin();
    return 0;
}
