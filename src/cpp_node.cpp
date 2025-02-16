// publisher_node.cpp
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class PublisherNode : public rclcpp::Node
{
public:
  PublisherNode() : Node("publisher_node")
  {
    // Create publisher
    publisher_ = this->create_publisher<std_msgs::msg::String>("trial", 10);
    
    // Create timer for periodic publishing (1 second interval)
    timer_ = this->create_wall_timer(
      1000ms, std::bind(&PublisherNode::timer_callback, this));
    
    RCLCPP_INFO(this->get_logger(), "Publisher Node started");
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "hello from ros2";
    publisher_->publish(message);
    RCLCPP_INFO(this->get_logger(), "Publishing: %s", message.data.c_str());
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PublisherNode>());
  rclcpp::shutdown();
  return 0;
}