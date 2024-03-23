#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "mujoco/mujoco.h"
// // #include "UIUX.c"
// #include "../include/simulate/simulate.h"
// #include "../include/simulate/glfw_adapter.h"
// #include "../include/simulate/array_safety.h"
#include "../include/simulate/functions.h"
using std::placeholders::_1;




mjModel* step_m = nullptr; // The MuJoCo model
mjData* step_d = nullptr;  // The data structure for simulation
const char* modelname = "/home/va/om/step/mujoco_ros2_humble_integrate/src/pkg1/src/MARKIV.xml";

void init_mujoco() {
  char error[1000] = "Could not load model";
  step_m = mj_loadXML(modelname, nullptr, error, 1000);
  if (!step_m) {
      std::cerr << error << std::endl;
      exit(1);
  }
  step_d = mj_makeData(step_m);
}

void myCallback(const mjModel* m, mjData* d)
{
    std::cout<<"data.act: " <<d->act[0] << std::endl;
    // std::cout<<"data.actforce: " <<d->actuator_force[0] << std::endl;
    // for (int i = 0; i < m->nbody; i++) {
    //     std::cout << "Body " << i << " position: "
    //               << d->xpos[i * 3] << ", "
    //               << d->xpos[i * 3 + 1] << ", "
    //               << d->xpos[i * 3 + 2] << std::endl;
    // }
}


#include "GLFW/glfw3.h"
// MuJoCo data structures
mjvCamera cam;                      // abstract camera
mjvOption opt;                      // visualization options
mjvScene scn;                       // abstract scene
mjrContext con;                     // custom GPU context
GLFWwindow* window;

void init_renderer() {
  // ... load model and data

  // init GLFW, create window, make OpenGL context current, request v-sync
  glfwInit();
  window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  // initialize visualization data structures
  mjv_defaultCamera(&cam);
  // mjv_defaultPerturb(&pert);
  mjv_defaultOption(&opt);
  mjr_defaultContext(&con);

  // create scene and context
  mjv_makeScene(step_m, &scn, 1000);
  mjr_makeContext(step_m, &con, mjFONTSCALE_100);



}
void loop_renderer(){
  // ... install GLFW keyboard and mouse callbacks

  // run main loop, target real-time simulation and 60 fps rendering
  // while( !glfwWindowShouldClose(window) ) {
    // advance interactive simulation for 1/60 sec
    //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
    //  this loop will finish on time for the next frame to be rendered at 60 fps.
    //  Otherwise add a cpu timer and exit this loop when it is time to render.
    mjtNum simstart = step_d->time;
    // while( d->time - simstart < 1.0/60.0 ){
      // for (int i = 0; i < m->nu; ++i) {
      //     // d->ctrl[i] = 80;
      //     std::cout<<"d->ctrl[i] is "<<d->ctrl[i]<<std::endl;
      // }
      mj_step(step_m, step_d);
    // }

    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

    mjv_updateScene(step_m, step_d, &opt, NULL, &cam, mjCAT_ALL, &scn);
    mjr_render(viewport, &scn, &con);

    glfwSwapBuffers(window);

    glfwPollEvents();
  // }
}
void clean_renderer(){
  glfwTerminate();
  mjv_freeScene(&scn);
  mjr_freeContext(&con);  
}
void simulation_step() {
    // Example: simple simulation step
    mj_step(step_m, step_d);

    // Handle GLFW events
    glfwPollEvents();
}



class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    // callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive,true);
    // options_.callback_group = callback_group_;


  }
  void init_sub(mj::Simulate* sim){


    init_mujoco();
    init_renderer();

    // auto sim = std::make_unique<mj::Simulate>(
    //     std::make_unique<mj::GlfwAdapter>(),
    //     &cam, &opt, &pert, /* is_passive = */ false
    // );

    std::cout<<"Subscriber Initialise";

    // std::thread physicsthreadhandle(&PhysicsThread, sim, modelname);
    // // start simulation UI loop (blocking call)
    // sim->RenderLoop();
    // physicsthreadhandle.join();


    auto topic_callback =
      [this](geometry_msgs::msg::Twist msg) -> void {
        // RCLCPP_INFO(this->get_logger(), "Linear: '(%f,%f,%f)'", msg.linear.x,msg.linear.z,msg.linear.z);
        RCLCPP_INFO(this->get_logger(), "Q: '%f'", msg.linear.x);  
        for (int i = 0; i < step_m->nu; ++i) {
            step_d->ctrl[i] = msg.linear.x;
            // std::cout<<"d->ctrl[i] is "<<d->ctrl[i]<<std::endl;
        }     
      };

     subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "turtle1/cmd_vel", 10, topic_callback);

    auto reg_callback =
      [this](geometry_msgs::msg::Twist msg) -> void {
        RCLCPP_INFO(this->get_logger(), "FRAME: '%f'", msg.linear.x);  
        loop_renderer();  
      };

     regulation_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "turtle1/frames", 10, reg_callback);   
  }
private:
  // void topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const
  // {
  //   RCLCPP_INFO(this->get_logger(), "Received - Linear Velocity x: '%f', Linear Velocity y: '%f', Angular Velocity: '%f'",
  //               msg->linear.x, msg->linear.y, msg->angular.z);
  // }
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr regulation_;
  // rclcpp::CallbackGroup::SharedPtr callback_group_;
  // rclcpp::SubscriptionOptions::SharedPtr options_;
};

int main(int argc, char* argv[]) {

  std::printf("MuJoCo version %s\n", mj_versionString());
  if (mjVERSION_HEADER!=mj_version()) {
    mju_error("Headers and library have different versions");
  }

  // scan for libraries in the plugin directory to load additional plugins
  // scanPluginLibraries();

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


  // INTIALISE INTERACTIVE UI
  std::cout<<"Subscriber Initialise";

  std::thread physicsthreadhandle(&PhysicsThread, sim.get(), modelname);
  // start simulation UI loop (blocking call)
  sim->RenderLoop();
  physicsthreadhandle.join();


  // INITIALISE ROS SUBSCRIBER
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MinimalSubscriber>();
  // mjcb_control = myCallback;
  // rclcpp::SubscriptionOptions options;
  // options.callback_group = my_callback_group;


  node->init_sub(sim.get());
  rclcpp::spin(node);








    // my_callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);






    rclcpp::shutdown();
    
    clean_renderer();



    mj_deleteData(step_d);
    mj_deleteModel(step_m);

    return 0;
}