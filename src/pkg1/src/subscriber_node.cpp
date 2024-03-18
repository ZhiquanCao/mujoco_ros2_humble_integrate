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
  char error[1000] = "Could not load model";
  m = mj_loadXML("/home/zhiquan/mujoco_ros2_humble_integrate/src/pkg1/src/7dof_arm.xml", nullptr, error, 1000);
  if (!m) {
      std::cerr << error << std::endl;
      exit(1);
  }
  d = mj_makeData(m);
}

void myCallback(const mjModel* m, mjData* d)
{
    std::cout<<"data.act: " <<d->act[0] << std::endl;
    std::cout<<"data.actforce: " <<d->actuator_force[0] << std::endl;
    for (int i = 0; i < m->nbody; i++) {
        std::cout << "Body " << i << " position: "
                  << d->xpos[i * 3] << ", "
                  << d->xpos[i * 3 + 1] << ", "
                  << d->xpos[i * 3 + 2] << std::endl;
    }
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
    while( d->time - simstart < 1.0/60.0 ){
      for (int i = 0; i < m->nu; ++i) {
          d->ctrl[i] = 50;
      }
      mj_step(m, d);
    }

    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

    mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
    mjr_render(viewport, &scn, &con);

    glfwSwapBuffers(window);

    glfwPollEvents();
  }

  glfwTerminate();
  mjv_freeScene(&scn);
  mjr_freeContext(&con);
}

void simulation_step() {
    // Example: simple simulation step
    mj_step(m, d);

    // Handle GLFW events
    glfwPollEvents();
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MinimalSubscriber>();
    
    init_mujoco();
    init_renderer();

    mjcb_control = myCallback;

    rclcpp::spin(node);
    rclcpp::shutdown();

    mj_deleteData(d);
    mj_deleteModel(m);

    return 0;
}