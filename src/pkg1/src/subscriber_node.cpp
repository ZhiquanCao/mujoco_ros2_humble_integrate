#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "mujoco/mujoco.h"

#include "../include/simulate/functions.h"
using std::placeholders::_1;


mjModel* step_m = nullptr; // The MuJoCo model
mjData* step_d = nullptr;  // The data structure for simulation
// const char* modelname = "/home/zhiquan/mujoco_ros2_humble_integrate/src/pkg1/src/6dof_from_hip.xml";
const char* modelname = "/home/zhiquan/mujoco_ros2_humble_integrate/src/pkg1/src/MARKIV.xml";


#include "GLFW/glfw3.h"
// MuJoCo data structures
mjvCamera cam;                      // abstract camera
mjvOption opt;                      // visualization options
mjvScene scn;                       // abstract scene
mjrContext con;                     // custom GPU context
GLFWwindow* window;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    subscription_ = this->create_subscription<geometry_msgs::msg::Polygon>(
      "turtle1/cmd_vel", 10, [this](const geometry_msgs::msg::Polygon::SharedPtr msg){this->topic_callback(msg);});
  }
  
private:
  void topic_callback(geometry_msgs::msg::Polygon::SharedPtr msg){
        RCLCPP_INFO(this->get_logger(), "Q: '%f'", msg->points[0].x); 

        // const std::unique_lock<std::recursive_mutex> lock(sim->mtx);

        for (int i = 0; i < m->nu; ++i) {
            d->ctrl[i] = msg->points[i].x;
        } 
      };
  rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr regulation_;

};

void RosThread(std::shared_ptr<MinimalSubscriber> node) {
  //node->init_sub(sim.get());
  rclcpp::spin(node);
}

int main(int argc, char* argv[]) {
  // PREPARE SIMUALTION WITH UI
  mjvCamera cam;
  mjv_defaultCamera(&cam);

  mjvOption opt;
  mjv_defaultOption(&opt);

  mjvPerturb pert;
  mjv_defaultPerturb(&pert);

  auto sim = std::make_unique<mj::Simulate>(
      std::make_unique<mj::GlfwAdapter>(),
      &cam, &opt, &pert, /* is_passive = */ false
  );

  rclcpp::init(argc, argv);
  auto node = std::make_shared<MinimalSubscriber>();

  std::thread physicsThreadHandle(&PhysicsThread, sim.get(), modelname);
  std::thread rosThreadHandle(&RosThread, node);
  sim->RenderLoop();

  // scan for libraries in the plugin directory to load additional plugins
  // scanPluginLibraries();
  std::cout<< "joining threads" <<std::endl;
  rosThreadHandle.join();
  physicsThreadHandle.join();
  rclcpp::shutdown();

  //sim->RenderLoopClean();

  // clean_renderer();
  //mj_deleteData(step_d);
  //mj_deleteModel(step_m);

  return 0;
}