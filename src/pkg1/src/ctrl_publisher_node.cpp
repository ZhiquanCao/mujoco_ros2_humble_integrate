#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "pkg1/msg/imu_data.hpp"
#include "pkg1/msg/ik6_dof.hpp"
using namespace std::chrono_literals;

#define M_PI 3.14159265358979323846



double deg_to_rad(double degree) { return degree * M_PI / 180; }

class ControlPublisher : public rclcpp::Node
{
  public:
    ControlPublisher()
    : Node("control_publisher")
    {
      publisher_ = this->create_publisher<pkg1::msg::IK6DOF>("/joint_pos_controller/joint_pos", 1);
      i = 0;
      timer_ = this->create_wall_timer(200ms, std::bind(&ControlPublisher::publish_message, this));
    }

  private:
    
    double left_thigh_ = 0.2;
    double left_knee_ = 2* left_thigh_;
    double left_ankle_ = -0.32;
    double right_thigh_ = -left_thigh_; 
    double right_knee_ = 2* left_thigh_; // 0.2
    double right_ankle_ = -0.32;
    double left_hip_ = deg_to_rad(2);
    double right_hip_ = deg_to_rad(2);

    //l_thigh, l_knee, r_thigh, r_knee, l_hip, r_hip
    double walk_trajs[4][6] ={
    {  left_thigh_,  left_knee_,   0.0,  0.0,  -left_hip_,  0.0  },
    {  -0.0,   0.0,  0.0,  0.0,  0.0,  0.0  },
    {  -0.0,   0.0,  right_thigh_,   right_knee_,  left_hip_,  -right_hip_  },
    {  -0.0,   0.0,  0.0,  0.0,  0.0,  0.0  }
    };

    void publish_message()
    {
      auto message = pkg1::msg::IK6DOF();

      message.left_thigh = walk_trajs[i][0];
      message.left_knee = -walk_trajs[i][1];
      message.left_ankle = -message.left_knee;
      message.right_thigh = walk_trajs[i][2];
      message.right_knee = walk_trajs[i][3];
      message.right_ankle = -message.right_knee;
      message.left_hip = walk_trajs[i][4];
      message.right_hip = walk_trajs[i][5];

      RCLCPP_INFO(this->get_logger(), "Sending LΔ : ('%f','%f','%f','%f')", message.left_thigh,message.left_knee,message.left_hip);
      RCLCPP_INFO(this->get_logger(), "Sending RΔ : ('%f','%f','%f','%f')", message.right_thigh,message.right_knee,message.right_hip);
      publisher_->publish(message);
      i ++;
      i %= 4; 
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<pkg1::msg::IK6DOF>::SharedPtr publisher_;
    int i;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlPublisher>());
  rclcpp::shutdown();
  return 0;
}