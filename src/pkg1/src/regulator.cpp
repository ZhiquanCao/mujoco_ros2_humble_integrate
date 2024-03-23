#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class FramePublisher : public rclcpp::Node
{
  public:
    FramePublisher()
    : Node("publisher")
    {
      publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/frames", 1);
      i = 0.0;
      timer_ = this->create_wall_timer(10ms, std::bind(&FramePublisher::publish_message, this));
    }

  private:
    void publish_message()
    {
      auto message = geometry_msgs::msg::Twist();
      message.linear.x = i;
      RCLCPP_INFO(this->get_logger(), "Sending - FRAME x: '%f'", message.linear.x);
      publisher_->publish(message);
      i += 1; 
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    float i;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FramePublisher>());
  rclcpp::shutdown();
  return 0;
}