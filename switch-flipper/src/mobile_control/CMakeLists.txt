cmake_minimum_required(VERSION 3.8)
project(mobile_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)


find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(interbotix_xs_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(trajectory_msgs REQUIRED)


add_executable(camera_waver src/camera_waver.cpp)

ament_target_dependencies(
  camera_waver
  rclcpp
  std_msgs
  geometry_msgs
  moveit_ros_planning_interface  
  tf2 
  tf2_ros
  trajectory_msgs
  interbotix_xs_msgs
  nav_msgs
  )


install(TARGETS camera_waver DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY
  launch
  # meshes
  config # urdf and srdf are under config
  meshes

  DESTINATION share/${PROJECT_NAME}/)



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

ament_package()
