#include <chrono>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class SerialTalker : public rclcpp::Node {
 public:
  SerialTalker() : Node("serial_talker"), count_(0) {
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&SerialTalker::cmd_vel_callback, this, std::placeholders::_1));
    string_pub_ = this->create_publisher<std_msgs::msg::String>("demo_topic", 10);
    auto timer_callback =
        [this]() -> void {
      auto message = std_msgs::msg::String();
      message.data = "Hello, world! " + std::to_string(this->count_++);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      this->string_pub_->publish(message);
    };
    timer_ = this->create_wall_timer(500ms, timer_callback);
  }

 private:
  void cmd_vel_callback(const geometry_msgs::msg::Twist& msg) {
    auto direction = std_msgs::msg::String();
    std::string msg_str = "";

    if (msg.angular.z > 0.5) {
      msg_str += "左";
    } else if (msg.angular.z < -0.5) {
      msg_str += "右";
    }
    if (msg.linear.x > 0.1) {
      msg_str += "前";
    } else if (msg.linear.x < -0.1) {
      msg_str += "後";
    }

    direction.data = msg_str;
    if (msg_str == "")
      return;
    string_pub_->publish(direction);
    RCLCPP_INFO(this->get_logger(), "Direction: %s", direction.data.c_str());
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SerialTalker>());
  rclcpp::shutdown();
  return 0;
}