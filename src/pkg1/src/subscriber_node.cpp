#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "mujoco/mujoco.h"

#include "../include/simulate/functions.h"
using std::placeholders::_1;


const char* modelname = "/home/zhiquan/mujoco_ros2_humble_integrate/src/pkg1/src/6dof_from_hip.xml";
// const char* modelname = "/home/zhiquan/mujoco_ros2_humble_integrate/src/pkg1/src/MARKIV.xml";

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber(mj::Simulate* sim_ptr)
  : Node("minimal_subscriber"), sim(sim_ptr)
  {
    subscription_ = this->create_subscription<geometry_msgs::msg::Polygon>(
      "turtle1/cmd_vel", 10, [this](const geometry_msgs::msg::Polygon::SharedPtr msg){this->topic_callback(msg);});
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

void RosThread(std::shared_ptr<MinimalSubscriber> node) {
  rclcpp::spin(node);
}

int main(int argc, char* argv[]) {
  // PREPARE SIMUALTION WITH UI
  mjvCamera cam;
  // mjv_defaultCamera(&cam); 

  cam.type = mjCAMERA_FREE;
  cam.lookat[0] = 0;    // x-coordinate of the point to look at
  cam.lookat[1] = 2000;    // y-coordinate of the point to look at
  cam.lookat[2] = 0;    // z-coordinate of the point to look at
  cam.distance = 200.0;   // distance from the lookat point
  cam.azimuth = 180;     // rotation around the vertical axis, in degrees
  cam.elevation = 0; 

  // m.cam_mode[0] = mjCAMERA_TRACKING; // Set the camera mode to tracking
  // m.cam_bodyid[0] = body_id_to_track; // Set the body ID the camera should track

  // // Optionally, if you want to manually set camera parameters
  // m.cam_pos[0] = x_position; // Set the X position
  // m.cam_pos[1] = y_position; // Set the Y position
  // m.cam_pos[2] = z_position; // Set the Z position

  mjvOption opt;
  mjv_defaultOption(&opt);

  mjvPerturb pert;
  mjv_defaultPerturb(&pert);

  auto sim = std::make_unique<mj::Simulate>(
      std::make_unique<mj::GlfwAdapter>(),
      &cam, &opt, &pert, /* is_passive = */ false
  );

  rclcpp::init(argc, argv);
  auto node = std::make_shared<MinimalSubscriber>(sim.get());

  std::thread physicsThreadHandle(&PhysicsThread, sim.get(), modelname);
  std::thread rosThreadHandle(&RosThread, node);
  sim->RenderLoop();

  // scan for libraries in the plugin directory to load additional plugins
  // scanPluginLibraries();
  std::cout<< "joining threads" <<std::endl;
  rosThreadHandle.join();
  physicsThreadHandle.join();
  rclcpp::shutdown();

  return 0;
}