#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/polygon.hpp"
// #include "trajectory_msgs/msg/multi_dof_joint_trajectory.hpp"
// #include "../msg/IK6DOF.msg"
using namespace std::chrono_literals;




float walk_trajs[24][6] = {
{  0,  0,  0,  0,  0,  0},
{  0,  0,  0,  0,  0.0,  0.0},
{  0,  0,  0,  0,  0.4,  0.4},
{  0,  0,  0,  0,  0.8,  0.8},
{  0,  0,  0,  0,  1.2,  1.2},
{  0,  0,  0,  0,  1.6,  1.6},
{  0,  0,  0,  0,  2.0,  2.0},
{  0,  0,  0,  0,  2.4,  2.4},
{  0,  0,  0,  0,  2.8,  2.8},
{  0,  0,  0,  0,  3.2,  3.2},
{  0,  0,  0,  0,  3.6,  3.6},
{  5,  0,  -5,   0,  4,  4},
{  0,  0,  0,  0,  0,  0},
{  0,  0,  0,  0,  0.0,  0.0},
{  0,  0,  0,  0,  -0.4,   -0.4},
{  0,  0,  0,  0,  -0.8,   -0.8},
{  0,  0,  0,  0,  -1.2,   -1.2},
{  0,  0,  0,  0,  -1.6,   -1.6},
{  0,  0,  0,  0,  -2.0,   -2.0},
{  0,  0,  0,  0,  -2.4,   -2.4},
{  0,  0,  0,  0,  -2.8,   -2.8},
{  0,  0,  0,  0,  -3.2,   -3.2},
{  0,  0,  0,  0,  -3.6,   -3.6},
{  7,  0,  -7,   0,  -4,   -4}
};


class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("publisher")
    {
      publisher_ = this->create_publisher<geometry_msgs::msg::Polygon>("turtle1/cmd_vel", 1);
      i = 0;
      timer_ = this->create_wall_timer(200ms, std::bind(&MinimalPublisher::publish_message, this));
    }

  private:
    void publish_message()
    {
      // auto p1 = geometry_msgs::msg::Point32();
      // p1.x = 1;
      // p1.y = 2;
      // p1.z = 3;
      std::vector<geometry_msgs::msg::Point32> q_vals(8);
      q_vals[0].x = -walk_trajs[i][0];
      q_vals[1].x = walk_trajs[i][0];
      q_vals[2].x = walk_trajs[i][1];
      q_vals[3].x = walk_trajs[i][4];

      q_vals[7].x = -walk_trajs[i][2];
      q_vals[6].x = walk_trajs[i][2];
      q_vals[5].x = walk_trajs[i][3];
      q_vals[4].x = walk_trajs[i][5];
      // q_vals[1].x = -2*sin(i);
      // q_vals[2].x = sin(i);
      // q_vals[3].x = cos(i);


      auto message = geometry_msgs::msg::Polygon();
      message.points = q_vals;
      // message.points[0] = p1;
      // message.y = sin(i);
      // message.x = sin(i);
      // message.x = sin(i);
      // message.x = sin(i);
      // message.x = sin(i);
      RCLCPP_INFO(this->get_logger(), "Sending LΔ : ('%f','%f','%f','%f')", message.points[0].x,message.points[1].x,message.points[2].x,message.points[3].x);
      RCLCPP_INFO(this->get_logger(), "Sending RΔ : ('%f','%f','%f','%f')", message.points[7].x,message.points[6].x,message.points[5].x,message.points[4].x);
      publisher_->publish(message);
      i += 1;
      i %= 24; 
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr publisher_;
    int i;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}