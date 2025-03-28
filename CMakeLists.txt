cmake_minimum_required(VERSION 3.16)
project(spbot_diffdrive LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# set the same behavior for windows as it is on linux
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_library(
  spbot_diffdrive
  SHARED
  hardware/diffbot_system.cpp
)
target_compile_features(spbot_diffdrive PUBLIC cxx_std_17)
target_include_directories(spbot_diffdrive PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/spbot_diffdrive>
)
ament_target_dependencies(
  spbot_diffdrive PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_link_libraries(spbot_diffdrive PUBLIC serial)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface spbot_diffdrive.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/spbot_diffdrive
)
install(
  DIRECTORY description/launch description/ros2_control description/urdf
  DESTINATION share/spbot_diffdrive
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/spbot_diffdrive
)
install(TARGETS spbot_diffdrive
  EXPORT export_spbot_diffdrive
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(example_2_urdf_xacro test/test_urdf_xacro.py)
  ament_add_pytest_test(view_example_2_launch test/test_view_robot_launch.py)
  ament_add_pytest_test(run_example_2_launch test/test_diffbot_launch.py)
endif()

## EXPORTS
ament_export_targets(export_spbot_diffdrive HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
