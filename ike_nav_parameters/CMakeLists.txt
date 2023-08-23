cmake_minimum_required(VERSION 3.8)
project(ike_nav_parameters)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

generate_parameter_library(ike_controller_parameter
  config/ike_controller_parameter.yaml
)

generate_parameter_library(ike_costmap_2d_parameter
  config/ike_costmap_2d_parameter.yaml
)

generate_parameter_library(ike_localization_parameter
  config/ike_localization_parameter.yaml
)

generate_parameter_library(ike_map_server_parameter
  config/ike_map_server_parameter.yaml
)

generate_parameter_library(ike_nav_server_parameter
  config/ike_nav_server_parameter.yaml
)

generate_parameter_library(ike_planner_parameter
  config/ike_planner_parameter.yaml
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_auto_package()