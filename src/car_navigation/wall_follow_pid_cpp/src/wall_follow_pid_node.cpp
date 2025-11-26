#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

namespace {
template <typename T>
T clamp(const T & value, const T & min_value, const T & max_value) {
  return std::max(min_value, std::min(value, max_value));
}
}  // namespace

class WallFollowPidNode : public rclcpp::Node {
public:
  WallFollowPidNode()
  : rclcpp::Node("wall_follow_pid_node"),
    integral_error_(0.0),
    previous_error_(0.0),
    previous_time_(0, 0, RCL_ROS_TIME)
  {
    declareParameters();

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan",
      rclcpp::SensorDataQoS(),
      std::bind(&WallFollowPidNode::scanCallback, this, std::placeholders::_1));

    drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
      "drive", rclcpp::QoS(10));
  }

private:
  void declareParameters() {
    kp_ = this->declare_parameter("kp", 0.8);
    ki_ = this->declare_parameter("ki", 0.0);
    kd_ = this->declare_parameter("kd", 0.1);
    integral_limit_ = this->declare_parameter("integral_limit", 1.0);

    target_distance_ = this->declare_parameter("desired_distance_from_wall", 1.0);
    lookahead_distance_ = this->declare_parameter("lookahead_distance", 3.0);
    theta_degrees_ = this->declare_parameter("theta_degrees", 60.0);
    theta_rad_ = theta_degrees_ * M_PI / 180.0;

    steering_angle_limit_ = this->declare_parameter("steering_angle_limit", 0.4);
    max_speed_ = this->declare_parameter("max_speed", 3.0);
    min_speed_ = this->declare_parameter("min_speed", 0.5);
    speed_reduction_gain_ = this->declare_parameter("speed_reduction_gain", 1.0);
    brake_speed_ = this->declare_parameter("brake_speed", 1.0);
    front_distance_threshold_ = this->declare_parameter("front_distance_threshold", 1.2);
    filter_distance_ = this->declare_parameter("filter_distance", 10.0);
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    if (msg->ranges.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Received empty LaserScan");
      return;
    }

    constexpr double angle_to_wall = -M_PI / 2.0;
    double b = getRange(*msg, angle_to_wall);
    double a = getRange(*msg, angle_to_wall + theta_rad_);

    double numerator = a * std::cos(theta_rad_) - b;
    double alpha = std::atan2(numerator, a * std::sin(theta_rad_));

    double ab = b * std::cos(alpha);
    double projected_distance = ab + lookahead_distance_ * std::sin(alpha);
    double error = target_distance_ - projected_distance;

    rclcpp::Time current_time(msg->header.stamp);
    if (current_time.nanoseconds() == 0) {
      current_time = this->get_clock()->now();
    }

    double control = computeControl(error, current_time);
    double steering_angle = clamp(control, -steering_angle_limit_, steering_angle_limit_);
    double speed = computeSpeed(steering_angle, *msg);

    ackermann_msgs::msg::AckermannDriveStamped drive_msg;
    drive_msg.header.stamp = current_time;
    drive_msg.header.frame_id = "base_link";
    drive_msg.drive.steering_angle = steering_angle;
    drive_msg.drive.speed = speed;
    drive_pub_->publish(drive_msg);
  }

  double computeSpeed(double steering_angle, const sensor_msgs::msg::LaserScan & scan) const {
    double speed = max_speed_ - speed_reduction_gain_ * std::fabs(steering_angle);
    speed = clamp(speed, min_speed_, max_speed_);

    double front_distance = getRange(scan, 0.0);
    if (front_distance < front_distance_threshold_) {
      speed = std::min(speed, brake_speed_);
    }
    return std::max(speed, 0.0);
  }

  double computeControl(double error, const rclcpp::Time & current_time) {
    double dt = 0.0;
    if (previous_time_.nanoseconds() > 0) {
      dt = (current_time - previous_time_).seconds();
    }

    if (dt > 0.0) {
      integral_error_ += error * dt;
      integral_error_ = clamp(integral_error_, -integral_limit_, integral_limit_);
    }

    double derivative = 0.0;
    if (dt > 1e-3) {
      derivative = (error - previous_error_) / dt;
    }

    double control = kp_ * error + ki_ * integral_error_ + kd_ * derivative;

    previous_time_ = current_time;
    previous_error_ = error;

    return control;
  }

  double getRange(const sensor_msgs::msg::LaserScan & scan, double angle) const {
    double bounded_angle = clamp(angle, static_cast<double>(scan.angle_min), static_cast<double>(scan.angle_max));
    double raw_index = (bounded_angle - scan.angle_min) / scan.angle_increment;
    int index = static_cast<int>(std::round(raw_index));
    index = clamp(index, 0, static_cast<int>(scan.ranges.size()) - 1);

    double distance = scan.ranges.at(static_cast<size_t>(index));
    if (!std::isfinite(distance) || distance < scan.range_min || distance > scan.range_max) {
      return filter_distance_;
    }
    return distance;
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;

  double integral_error_;
  double previous_error_;
  rclcpp::Time previous_time_;

  double steering_angle_limit_;
  double max_speed_;
  double min_speed_;
  double speed_reduction_gain_;
  double brake_speed_;
  double front_distance_threshold_;
  double filter_distance_;

  double lookahead_distance_;
  double target_distance_;
  double theta_degrees_;
  double theta_rad_;

  double kp_;
  double ki_;
  double kd_;
  double integral_limit_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WallFollowPidNode>());
  rclcpp::shutdown();
  return 0;
}