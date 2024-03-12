#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "mujoco/mujoco.h"

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "turtle1/cmd_vel", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Received - Linear Velocity x: '%f', Linear Velocity y: '%f', Angular Velocity: '%f'",
                msg->linear.x, msg->linear.y, msg->angular.z);
  }
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
};

mjModel* m = nullptr; // The MuJoCo model
mjData* d = nullptr;  // The data structure for simulation

void init_mujoco() {
  
  // Load the model
  char error[1000] = "Could not load model";
  m = mj_loadXML("/home/zhiquan/mujoco_ros2_humble_integrate/src/pkg1/src/hello.xml", nullptr, error, 1000);
  if (!m) {
      std::cerr << error << std::endl;
      exit(1);
  }
  d = mj_makeData(m);
}

#include "GLFW/glfw3.h"
mjvCamera cam;                   // camera
mjvScene scn;                    // abstract scene
mjrContext con;                  // custom GPU context

void init_renderer() {
    // Initialize GLFW
    if (!glfwInit()) {
        std::cerr << "Could not initialize GLFW." << std::endl;
        exit(1);
    }

    // Create a window and make its context current (using GLFW)
    GLFWwindow* window = glfwCreateWindow(1200, 900, "MuJoCo Simulation", nullptr, nullptr);
    if (!window) {
        std::cerr << "Could not create GLFW window." << std::endl;
        exit(1);
    }
    glfwMakeContextCurrent(window);

    // Initialize MuJoCo visualization contexts
    mjv_defaultCamera(&cam);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);

    // Assign the MuJoCo model to the scene
    mjv_makeScene(m, &scn, 2000);   // 2000 is the max number of geom objects to be visualized
    mjr_makeContext(m, &con, mjFONTSCALE_100);
}

void simulation_step() {
    // Example: simple simulation step
    mj_step(m, d);

    // Optional: Render the scene
    // mjr_render(&rect, &scn, &con);
    
    // Handle GLFW events
    glfwPollEvents();
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MinimalSubscriber>();
    
    // Initialize MuJoCo and rendering
    init_mujoco();
    init_renderer();

    // Main ROS loop in a separate thread or using a ROS 2 timer
    std::thread mujoco_thread([&] {
        while (rclcpp::ok()) {
            simulation_step();
        }
    });

    rclcpp::spin(node);
    rclcpp::shutdown();

    // Cleanup
    mj_deleteData(d);
    mj_deleteModel(m);

    return 0;
}