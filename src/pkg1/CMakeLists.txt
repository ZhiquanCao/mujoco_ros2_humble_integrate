cmake_minimum_required(VERSION 3.8)
project(pkg1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(glfw3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/IK6DOF.msg"
  "msg/ImuData.msg"
)

set(HOME_DIR $ENV{HOME})
set(MUJOCO_DIR "${HOME_DIR}/.mujoco/mujoco-3.1.2")

include_directories("${MUJOCO_DIR}/include/")
include_directories("./include/")

find_library(MUJOCO_LIBRARY mujoco PATHS ${HOME_DIR}/.mujoco/mujoco-3.1.2/lib) 


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

endif()


FILE(GLOB c_includes include/simulate/*.cc)

set(dependencies "geometry_msgs"  "trajectory_msgs" "rclcpp" )
add_executable(ctrl_pub src/ctrl_publisher_node.cpp)
add_executable(sim_sub src/sim_subscriber_node.cpp ${c_includes})
ament_target_dependencies(ctrl_pub ${dependencies})
ament_target_dependencies(sim_sub ${dependencies})
rosidl_target_interfaces(ctrl_pub ${PROJECT_NAME} "rosidl_typesupport_cpp") # Where to find the generated struct file for defined msg; used along with #include "pkg1/msg/imu.hpp"
rosidl_target_interfaces(sim_sub ${PROJECT_NAME} "rosidl_typesupport_cpp") # Where to find the generated struct file for defined msg; used along with #include "pkg1/msg/imu.hpp"

target_link_libraries(sim_sub ${MUJOCO_LIBRARY} glfw)


install(TARGETS
  ctrl_pub
  sim_sub
  DESTINATION lib/${PROJECT_NAME})

ament_package()
