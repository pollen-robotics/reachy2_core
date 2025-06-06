cmake_minimum_required(VERSION 3.8)
project(dynamixel_2joints_system_hwi)

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
  src/dynamixel_2joints_system_hwi.cpp
)
target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
  ${CMAKE_SOURCE_DIR}/../dynamixel_c_api/
)

add_library(dynamixel_c_api STATIC IMPORTED)
set_property(TARGET dynamixel_c_api PROPERTY IMPORTED_LOCATION $ENV{PWD}/../../install/dynamixel_c_api/lib/dynamixel_c_api/libdynamixel_c_api.a )

target_link_libraries(
  ${PROJECT_NAME}
  dynamixel_c_api
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

pluginlib_export_plugin_description_file(hardware_interface dynamixel_2joints_system_hwi.xml)

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
