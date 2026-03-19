#include <sstream>
#include <string>

#include "ros/ros.h"
#include "std_msgs/Int16MultiArray.h"

namespace {

void bumpCallback(const std_msgs::Int16MultiArray::ConstPtr& msg) {
  std::ostringstream oss;
  oss << "Bump Sensor Data: ";
  for (std::size_t i = 0; i < msg->data.size(); ++i) {
    if (i != 0) oss << ", ";
    oss << "S" << i << "=" << (msg->data[i] ? "1" : "0");
  }

  // 以 1Hz 节流输出，避免高频刷屏；需要更频繁可调小这个值
  ROS_INFO_STREAM_THROTTLE(1.0, oss.str());
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "bump_sensor_subscriber");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string topic;
  int queue_size = 10;
  pnh.param<std::string>("topic", topic, "/robot/bump_sensor");
  pnh.param("queue_size", queue_size, 10);
  if (queue_size < 1) queue_size = 1;

  ros::Subscriber sub = nh.subscribe(topic, queue_size, bumpCallback);
  ros::spin();
  return 0;
}
