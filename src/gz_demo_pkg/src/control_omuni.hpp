#ifndef CONTROL_OMUNI_HPP
#define CONTROL_OMUNI_HPP

#include <Eigen/Dense>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float64.hpp"

namespace omuni_control {

class OmuniControl : public rclcpp::Node {
 public:
  OmuniControl(const rclcpp::NodeOptions& options);

 private:
  void cmd_callback(const geometry_msgs::msg::Twist& msg);
  void joy_callback(const sensor_msgs::msg::Joy& msg);
  std::vector<float> omuni_controller(const float x_v, const float y_v, const float& anglar_v);

  std::vector<std::string> topic_names = {"/whl_lf_v", "/whl_lb_v", "/whl_rb_v", "/whl_rf_v"};

  std::vector<double> whl_yaw = {1.0 / 4.0 * M_PI, 3.0 / 4.0 * M_PI, -3.0 / 4.0 * M_PI, -1.0 / 4.0 * M_PI};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> whl_publishers_;
};

}  // namespace omuni_control

#endif