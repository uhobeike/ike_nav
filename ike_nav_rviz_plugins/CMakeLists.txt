cmake_minimum_required(VERSION 3.8)
project(ike_nav_rviz_plugins)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Qt5 REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
add_compile_definitions(QT_NO_KEYWORDS)

set(CMAKE_AUTOUIC_SEARCH_PATHS src/ui)
qt5_wrap_ui(UIC_FILES
  src/ui/ike_nav.ui
)

include_directories(${CMAKE_CURRENT_BINARY_DIR})

ament_auto_add_library(${PROJECT_NAME} SHARED
  include/ike_nav_rviz_plugins/ike_nav_panel.hpp
  include/ike_nav_rviz_plugins/waypoint_set_tool.hpp
  include/ike_nav_rviz_plugins/waypoints_display.hpp
  include/ike_nav_rviz_plugins/waypoints_visual.hpp
  src/ike_nav_panel.cpp
  src/waypoint_set_tool.cpp
  src/waypoints_display.cpp
  src/waypoints_visual.cpp
  ${UIC_FILES}
)

target_link_libraries(${PROJECT_NAME}
  ${QT_LIBRARIES}
)

pluginlib_export_plugin_description_file(rviz_common plugins/plugin_description.xml)
register_rviz_ogre_media_exports(DIRECTORIES "media")

ament_auto_package(
  INSTALL_TO_SHARE
  icons
  media
  plugins
)
