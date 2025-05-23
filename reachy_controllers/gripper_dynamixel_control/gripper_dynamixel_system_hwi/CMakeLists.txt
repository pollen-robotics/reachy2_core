cmake_minimum_required(VERSION 3.8)
project(gripper_dynamixel_system_hwi)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(
  ${PROJECT_NAME}
  SHARED
  src/gripper_dynamixel_system_hwi.cpp
)
target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
  ${CMAKE_SOURCE_DIR}/../gripper_dynamixel_controller/
)

add_library(gripper_dynamixel_controller STATIC IMPORTED)
set_property(TARGET gripper_dynamixel_controller PROPERTY IMPORTED_LOCATION $ENV{PWD}/../../install/gripper_dynamixel_controller/lib/gripper_dynamixel_controller/libgripper_dynamixel_controller.a )

target_link_libraries(
  ${PROJECT_NAME}
  gripper_dynamixel_controller
  udev
  gcc_s
  util
  rt
  pthread
  m
  dl
  c
 )

 ament_target_dependencies(
  ${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(hardware_interface gripper_dynamixel_system_hwi.xml)

# INSTALL
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
