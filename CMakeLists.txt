cmake_minimum_required(VERSION 3.8)
project(apds9960)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ColorProximity.msg"
  DEPENDENCIES std_msgs
)

add_executable(apds9960_node src/apds9960.cpp
                        lib/libi2c/src/i2c.c
                        lib/driver/src/driver_apds9960.c)
ament_target_dependencies(apds9960_node rclcpp std_msgs)
target_include_directories(apds9960_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${CMAKE_CURRENT_SOURCE_DIR}/include/apds9960
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/libi2c/include
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/driver/src)
target_compile_features(apds9960_node PUBLIC c_std_99 cxx_std_20)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(apds9960_node "${cpp_typesupport_target}")

install(TARGETS apds9960_node
  DESTINATION lib/${PROJECT_NAME})

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

ament_export_dependencies(rosidl_default_runtime)
ament_package()
