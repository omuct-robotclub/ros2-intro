cmake_minimum_required(VERSION 3.8)
project(gz_demo_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(points_integration SHARED src/points_integration.cpp)
ament_target_dependencies(points_integration
  rclcpp rclcpp_components 
  sensor_msgs
  laser_geometry
  tf2_geometry_msgs
  tf2_ros tf2
  pcl_ros pcl_conversions
)

rclcpp_components_register_node(points_integration PLUGIN "points_processes::PointIntegration" EXECUTABLE points_integration_node)

ament_export_targets(export_component)
install(TARGETS points_integration
        EXPORT export_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

add_library(odom_icp SHARED src/ICP.cpp src/icp_odom.cpp)
ament_target_dependencies(odom_icp
  rclcpp rclcpp_components 
  sensor_msgs
  laser_geometry
  tf2_geometry_msgs tf2_ros tf2
  pcl_ros pcl_conversions
  nav_msgs
)
rclcpp_components_register_node(
 odom_icp
 PLUGIN "icp_odom::IcpOdom"
 EXECUTABLE odom_icp_node
)
ament_export_targets(export_component)
install(TARGETS odom_icp
        EXPORT export_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

add_library(omuni_control SHARED src/control_omuni.cpp)

ament_target_dependencies(omuni_control
  rclcpp rclcpp_components
  geometry_msgs std_msgs sensor_msgs
  Eigen3
)

rclcpp_components_register_node(
 omuni_control
 PLUGIN "omuni_control::OmuniControl"
 EXECUTABLE omuni_control_node
)

ament_export_targets(export_component)
install(TARGETS omuni_control
        EXPORT export_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

include_directories(${EIGEN3_INCLUDE_DIR})

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

install(
  DIRECTORY launch config description
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
