cmake_minimum_required(VERSION 3.8)
project(ike_nav_msgs)

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/NavigateToGoal.action"
  "msg/function/VariableWaypointRadius.msg"
  "msg/Function.msg"
  "msg/Waypoint.msg"
  "msg/Waypoints.msg"
  "srv/GetCostMap2D.srv"
  "srv/GetMap.srv"
  "srv/GetPath.srv"
  "srv/GetTwist.srv"
  "srv/GetWaypointsMsg.srv"
  "srv/LoadWaypointYaml.srv"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs nav_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

ament_export_dependencies(rosidl_default_runtime)

ament_auto_package()
