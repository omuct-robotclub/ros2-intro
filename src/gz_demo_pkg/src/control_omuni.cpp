#include "control_omuni.hpp"

namespace omuni_control {

OmuniControl::OmuniControl(const rclcpp::NodeOptions& options) : rclcpp::Node("omuni_node", options) {
  this->declare_parameter<std::string>("mode", "joy");
  std::string mode = this->get_parameter("mode").as_string();
  for (const auto& name : topic_names) whl_publishers_.push_back(this->create_publisher<std_msgs::msg::Float64>(name, 10));

  if (mode == "cmd") {
    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10, std::bind(&OmuniControl::cmd_callback, this, std::placeholders::_1));
  } else if (mode == "joy") {
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 10, std::bind(&OmuniControl::joy_callback, this, std::placeholders::_1));
  } else if (mode == "nav") {
    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel_nav", 10, std::bind(&OmuniControl::cmd_callback, this, std::placeholders::_1));
  } else {
    RCLCPP_WARN(this->get_logger(), "mode is invalid. It must be 'joy' or 'cmd'.");
  }
}

void OmuniControl::cmd_callback(const geometry_msgs::msg::Twist& msg) {
  std::vector<float> rotations;
  rotations = omuni_controller(msg.linear.y, -msg.linear.x, msg.angular.z);
  for (size_t i = 0; i < whl_publishers_.size(); i++) {
    std_msgs::msg::Float64 msg;
    msg.data = rotations[i];
    whl_publishers_[i]->publish(msg);
  }
}

void OmuniControl::joy_callback(const sensor_msgs::msg::Joy& msg) {
  std::vector<float> rotations;
  rotations = omuni_controller(7.0 * msg.axes[0], 7.0 * msg.axes[1], 3.0 * msg.axes[3]);
  for (size_t i = 0; i < whl_publishers_.size(); i++) {
    std_msgs::msg::Float64 msg;
    msg.data = rotations[i];
    whl_publishers_[i]->publish(msg);
  }
}

std::vector<float> OmuniControl::omuni_controller(
    const float x_v, const float y_v, const float& anglar_v) {  // [m/s], [m/s], [rad/s]

  std::vector<float> rotations;

  // ホイール半径 [m]
  constexpr double R = 0.1;
  // ロボット中心からホイールまでの距離 [m]
  constexpr double L = 0.5;

  for (const double& th : whl_yaw) {
    double whl_ang_v = (1.0 / R) * (x_v * std::cos(th) + y_v * std::sin(th) + L * anglar_v);
    rotations.push_back(whl_ang_v);  // [rad/s]
  }
  return rotations;
}

}  // namespace omuni_control

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(omuni_control::OmuniControl)