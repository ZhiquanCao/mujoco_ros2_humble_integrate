#include <memory>
#include <stdexcept>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "mujoco/mujoco.h"
#include "../include/simulate/physics.h"
#include "pkg1/msg/ik6_dof.hpp"

using std::placeholders::_1;

#define LEFT_THIGH 0
#define LEFT_KNEE 1
#define LEFT_ANKLE 2
#define RIGHT_THIGH 3
#define RIGHT_KNEE 4
#define RIGHT_ANKLE 5
#define LEFT_HIP 6
#define RIGHT_HIP 7
class SimSubscriber : public rclcpp::Node
{
public:
  SimSubscriber(mj::Simulate* sim_ptr)
  : Node("sim_subscriber"), sim(sim_ptr)
  {
    subscription_ = this->create_subscription<pkg1::msg::IK6DOF>(
      "/joint_pos_controller/joint_pos", 10, [this](const pkg1::msg::IK6DOF::SharedPtr msg){this->topic_callback(msg);});
  }
  
private:
  void topic_callback(pkg1::msg::IK6DOF::SharedPtr msg){
        // RCLCPP_INFO(this->get_logger(), "Sending LΔ : (Thigh '%f', Knee'%f', Ankle'%f', Hip'%f')", msg.left_thigh, msg.left_knee, msg.left_ankle, msg.left_hip);
        const std::unique_lock<std::recursive_mutex> lock(sim->mtx);

        if (m != nullptr) {
          d->ctrl[LEFT_THIGH] = msg->left_thigh;
          d->ctrl[LEFT_KNEE] = msg->left_knee;
          d->ctrl[LEFT_ANKLE] = msg->left_ankle;
          d->ctrl[RIGHT_THIGH] = msg->right_thigh;
          d->ctrl[RIGHT_KNEE] = msg->right_knee;
          d->ctrl[RIGHT_ANKLE] = msg->right_ankle;
          d->ctrl[LEFT_HIP] = msg->left_hip;
          d->ctrl[RIGHT_HIP] = msg->right_hip;
        } else {
          RCLCPP_ERROR(this->get_logger(), "mjData is not properly initialized");
        }
        
      };
  mj::Simulate* sim;
  rclcpp::Subscription<pkg1::msg::IK6DOF>::SharedPtr subscription_;
};

void RosThread(std::shared_ptr<SimSubscriber> node) {
  rclcpp::spin(node);
}

int main(int argc, char* argv[]) {

  const char* homeDir = getenv("HOME");
  if (homeDir == nullptr) {
    throw std::runtime_error("HOME directory environment variable not set");
  }
  std::string concate_path = std::string(homeDir) + "/mujoco_ros2_humble_integrate/src/pkg1/src/6dof_from_hip.xml";
  const char* model_path = concate_path.c_str();

  mjvCamera cam;

  mjvOption opt;
  mjv_defaultOption(&opt);

  mjvPerturb pert;
  mjv_defaultPerturb(&pert);

  auto sim = std::make_unique<mj::Simulate>(
      std::make_unique<mj::GlfwAdapter>(),
      &cam, &opt, &pert, /* is_passive = */ false
  );

  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimSubscriber>(sim.get());

  std::thread physicsThreadHandle(&PhysicsThread, sim.get(), model_path);
  std::thread rosThreadHandle(&RosThread, node);
  sim->RenderLoop();

  rosThreadHandle.join();
  physicsThreadHandle.join();
  rclcpp::shutdown();

  return 0;
}