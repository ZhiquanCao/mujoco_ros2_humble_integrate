#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "mujoco/mujoco.h"

#include "../include/simulate/physics.h"
using std::placeholders::_1;


const char* modelname = "/home/zhiquan/mujoco_ros2_humble_integrate/src/pkg1/src/6dof_from_hip.xml";
// const char* modelname = "/home/zhiquan/mujoco_ros2_humble_integrate/src/pkg1/src/MARKIV.xml";

class SimSubscriber : public rclcpp::Node
{
public:
  SimSubscriber(mj::Simulate* sim_ptr)
  : Node("sim_subscriber"), sim(sim_ptr)
  {
    subscription_ = this->create_subscription<geometry_msgs::msg::Polygon>(
      "mark4/joints_pos", 10, [this](const geometry_msgs::msg::Polygon::SharedPtr msg){this->topic_callback(msg);});
  }
  
private:
  void topic_callback(geometry_msgs::msg::Polygon::SharedPtr msg){
        RCLCPP_INFO(this->get_logger(), "Q: '%f'", msg->points[0].x); 
        const std::unique_lock<std::recursive_mutex> lock(sim->mtx);
        if (m != nullptr) {
          for (int i = 0; i < m->nu; ++i) {
            d->ctrl[i] = msg->points[i].x;
          } 
        } else {
          RCLCPP_ERROR(this->get_logger(), "mjData is not properly initialized");
        }
      };
  mj::Simulate* sim;
  rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr subscription_;
};

void RosThread(std::shared_ptr<SimSubscriber> node) {
  rclcpp::spin(node);
}

int main(int argc, char* argv[]) {
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

  std::thread physicsThreadHandle(&PhysicsThread, sim.get(), modelname);
  std::thread rosThreadHandle(&RosThread, node);
  sim->RenderLoop();

  rosThreadHandle.join();
  physicsThreadHandle.join();
  rclcpp::shutdown();

  return 0;
}