#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <cmath>
#include <string>

static void rangeCallback(const sensor_msgs::Range::ConstPtr& msg,
                          const std::string& label,
                          double log_throttle_s) {
  const float r = msg->range;

  if (!std::isfinite(r)) {
    ROS_WARN_THROTTLE(log_throttle_s, "[%s] range is not finite", label.c_str());
    return;
  }

  // Respect msg-provided bounds when available.
  if (msg->min_range > 0.0f && msg->max_range > 0.0f &&
      (r < msg->min_range || r > msg->max_range)) {
    ROS_WARN_THROTTLE(log_throttle_s,
                      "[%s] out of bounds: %.3f (min=%.3f max=%.3f)",
                      label.c_str(), r, msg->min_range, msg->max_range);
    return;
  }

  ROS_INFO_THROTTLE(log_throttle_s, "Distance %s: %.3f", label.c_str(), r);
}

static ros::Subscriber subscribeRange(ros::NodeHandle& nh,
                                      const std::string& topic,
                                      const std::string& label,
                                      int queue_size,
                                      double log_throttle_s) {
  return nh.subscribe<sensor_msgs::Range>(
      topic, queue_size,
      boost::function<void(const sensor_msgs::Range::ConstPtr&)>(
          [label, log_throttle_s](const sensor_msgs::Range::ConstPtr& msg) {
            rangeCallback(msg, label, log_throttle_s);
          }));
}

int main(int argc, char** argv){
ros::init(argc, argv, "range_subscriber");
ros::NodeHandle nh;
ros::NodeHandle pnh("~");

std::string topic_left, topic_front, topic_right;
int queue_size = 10;
double log_throttle_s = 0.5;

pnh.param<std::string>("topic_left", topic_left, std::string("/ul/sensor1"));
pnh.param<std::string>("topic_front", topic_front, std::string("/ul/sensor2"));
pnh.param<std::string>("topic_right", topic_right, std::string("/ul/sensor3"));
pnh.param("queue_size", queue_size, queue_size);
pnh.param("log_throttle_s", log_throttle_s, log_throttle_s);

ros::Subscriber sub_1 = subscribeRange(nh, topic_left, "Left", queue_size, log_throttle_s);
ros::Subscriber sub_2 = subscribeRange(nh, topic_front, "Front", queue_size, log_throttle_s);
ros::Subscriber sub_3 = subscribeRange(nh, topic_right, "Right", queue_size, log_throttle_s);
ros::spin();
return 0;
}
