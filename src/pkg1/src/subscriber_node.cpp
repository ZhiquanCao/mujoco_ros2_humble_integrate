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
// MuJoCo data structures
mjvCamera cam;                      // abstract camera
mjvOption opt;                      // visualization options
mjvScene scn;                       // abstract scene
mjrContext con;                     // custom GPU context

void init_renderer() {
  // ... load model and data

  // init GLFW, create window, make OpenGL context current, request v-sync
  glfwInit();
  GLFWwindow* window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  // initialize visualization data structures
  mjv_defaultCamera(&cam);
  // mjv_defaultPerturb(&pert);
  mjv_defaultOption(&opt);
  mjr_defaultContext(&con);

  // create scene and context
  mjv_makeScene(m, &scn, 1000);
  mjr_makeContext(m, &con, mjFONTSCALE_100);

  // ... install GLFW keyboard and mouse callbacks

  // run main loop, target real-time simulation and 60 fps rendering
  while( !glfwWindowShouldClose(window) ) {
    // advance interactive simulation for 1/60 sec
    //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
    //  this loop will finish on time for the next frame to be rendered at 60 fps.
    //  Otherwise add a cpu timer and exit this loop when it is time to render.
    mjtNum simstart = d->time;
    while( d->time - simstart < 1.0/60.0 )
        mj_step(m, d);

    // get framebuffer viewport
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

    // update scene and render
    mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
    mjr_render(viewport, &scn, &con);

    // swap OpenGL buffers (blocking call due to v-sync)
    glfwSwapBuffers(window);

    // process pending GUI events, call GLFW callbacks
    glfwPollEvents();
  }

  // close GLFW, free visualization storage
  glfwTerminate();
  mjv_freeScene(&scn);
  mjr_freeContext(&con);

  // ... free MuJoCo model and data
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