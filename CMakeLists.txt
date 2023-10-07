cmake_minimum_required(VERSION 3.16)
project(tr_robot LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
set(HW_IF_INCLUDE_DEPENDS
  pluginlib
  rcpputils
  hardware_interface
)
set(REF_GEN_INCLUDE_DEPENDS
  kdl_parser
  rclcpp
  trajectory_msgs
)
set(CONTROLLER_INCLUDE_DEPENDS
  pluginlib
  rcpputils
  controller_interface
  realtime_tools
  trajectory_msgs
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${HW_IF_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
foreach(Dependency IN ITEMS ${REF_GEN_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
foreach(Dependency IN ITEMS ${CONTROLLER_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


## COMPILE
add_executable(send_trajectory reference_generator/send_trajectory.cpp)

ament_target_dependencies(
  send_trajectory PUBLIC
  ${REF_GEN_INCLUDE_DEPENDS}
)

add_library(
  tr_robot
  SHARED
  hardware/mercury_hardware.cpp
  hardware/ODrive-CAN/ODrive.cpp
  hardware/ODrive-CAN/CAN.cpp
  controller/mercury_controller.cpp
)

target_compile_features(tr_robot PUBLIC cxx_std_17)
target_include_directories(tr_robot PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/controller/include>
$<INSTALL_INTERFACE:include/tr_robot>
)
ament_target_dependencies(
  tr_robot PUBLIC
  ${HW_IF_INCLUDE_DEPENDS}
  ${CONTROLLER_INCLUDE_DEPENDS}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "ROS2_CONTROL_DEMO_EXAMPLE_7_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface tr_robot.xml)
# Export controller plugins
pluginlib_export_plugin_description_file(controller_interface tr_robot.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/tr_robot
)
install(
  DIRECTORY description/launch description/ros2_control description/urdf
  DESTINATION share/tr_robot
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/tr_robot
)

install(
    TARGETS send_trajectory
    RUNTIME DESTINATION lib/tr_robot
)

install(TARGETS tr_robot
  EXPORT export_ros2_control_demo_example_7
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

## EXPORTS
ament_export_targets(export_ros2_control_demo_example_7 HAS_LIBRARY_TARGET)
ament_export_dependencies(${HW_IF_INCLUDE_DEPENDS} ${REF_GEN_INCLUDE_DEPENDS} ${CONTROLLER_INCLUDE_DEPENDS})
ament_package()