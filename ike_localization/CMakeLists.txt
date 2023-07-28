cmake_minimum_required(VERSION 3.8)
project(ike_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/mcl/likelihoodField.cpp
  src/mcl/mcl.cpp
  src/mcl/motionModel.cpp
  src/mcl/observationModel.cpp
  src/mcl/resampling.cpp
  src/ike_localization.cpp
)

target_link_libraries(${PROJECT_NAME})

rclcpp_components_register_nodes(${PROJECT_NAME}
  PLUGIN "ike_nav::IkeLocalization"
  EXECUTABLE ${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
