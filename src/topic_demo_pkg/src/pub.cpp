#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"

class SerialTalker : public rclcpp::Node {
 public:
  SerialTalker() : Node("serial_talker") {
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&SerialTalker::cmd_vel_callback, this, std::placeholders::_1));
    string_pub_ = this->create_publisher<std_msgs::msg::String>("demo_topic", 10);
  }

 private:
  void cmd_vel_callback(const geometry_msgs::msg::Twist& msg) {
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SerialTalker>());
  rclcpp::shutdown();
  return 0;
}