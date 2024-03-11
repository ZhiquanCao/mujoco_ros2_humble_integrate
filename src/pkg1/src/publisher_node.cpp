#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("publisher")
    {
      publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 1);
      i = 0.0;
      timer_ = this->create_wall_timer(500ms, std::bind(&MinimalPublisher::publish_message, this));
    }

  private:
    void publish_message()
    {
      auto message = geometry_msgs::msg::Twist();
      message.linear.x = 1.0;
      message.linear.y = 0.55;
      message.angular.z = 0.5 + i;
      RCLCPP_INFO(this->get_logger(), "Sending - Linear Velocity x: '%f', Linear Velocity y: '%f', Angular Velocity : '%f'", message.linear.x, message.linear.y, message.angular.z);
      publisher_->publish(message);
      i += 0.1; 
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    float i;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}