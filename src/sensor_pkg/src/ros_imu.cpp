#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>

namespace {
double g_print_hz = 5.0; // 默认每秒打印 5 次
} // namespace

void imu_callback(const sensor_msgs::Imu::ConstPtr& imu_msg) {
  const geometry_msgs::Vector3& linear_acceleration = imu_msg->linear_acceleration;
  const geometry_msgs::Vector3& angular_velocity = imu_msg->angular_velocity;
  const geometry_msgs::Quaternion& orientation = imu_msg->orientation;

  const double period_sec = (g_print_hz > 0.0) ? (1.0 / g_print_hz) : 1.0;
  ROS_INFO_THROTTLE(period_sec,
                    "Linear acc: [%.6f %.6f %.6f]  Angular vel: [%.6f %.6f %.6f]  "
                    "Orientation(q): [%.6f %.6f %.6f %.6f]",
                    linear_acceleration.x, linear_acceleration.y, linear_acceleration.z,
                    angular_velocity.x, angular_velocity.y, angular_velocity.z,
                    orientation.x, orientation.y, orientation.z, orientation.w);
}
int main(int argc, char** argv) {
  ros::init(argc, argv, "imu_listener");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string imu_topic;
  int queue_size = 50;
  pnh.param<std::string>("imu_topic", imu_topic, std::string("/imu/data"));
  pnh.param("queue_size", queue_size, queue_size);
  pnh.param("print_hz", g_print_hz, g_print_hz);

  ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>(imu_topic, queue_size, imu_callback);
  ros::spin();
  return 0;
}
