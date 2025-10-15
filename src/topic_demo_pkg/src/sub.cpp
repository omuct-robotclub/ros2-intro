#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class demo_sub : public rclcpp::Node {
 public:
  demo_sub() : Node("demo_subscriber") {
    subscription_ = this->create_subscription<std_msgs::msg::String>("demo_topic", 10, std::bind(&demo_sub::topic_callback, this, std::placeholders::_1));
  }

 private:
  void topic_callback(const std_msgs::msg::String& msg) {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<demo_sub>());
  rclcpp::shutdown();
  return 0;
}