cmake_minimum_required(VERSION 3.8)
project(ike_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O3 -Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(Ceres REQUIRED)

ament_auto_find_build_dependencies()

include_directories(${CERESS_INCLUDE_DIRS})

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/ike_controller.cpp
)

target_link_libraries(${PROJECT_NAME} ${CERES_LIBRARIES})

rclcpp_components_register_nodes(${PROJECT_NAME}
  PLUGIN "ike_nav::IkeController"
  EXECUTABLE ${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
