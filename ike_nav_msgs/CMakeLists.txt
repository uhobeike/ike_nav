cmake_minimum_required(VERSION 3.8)
project(ike_nav_msgs)

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetCostMap2D.srv"
  "srv/GetMap.srv"
  "srv/GetPath.srv"
  "srv/GetTwist.srv"
  DEPENDENCIES builtin_interfaces std_msgs nav_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_auto_package()
