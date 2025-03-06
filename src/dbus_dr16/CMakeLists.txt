cmake_minimum_required(VERSION 3.8)
project(dbus_dr16)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(dbus_dr16_interface REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


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


add_library(dbus_dr16_lib src/dbus_dr16.cpp)
ament_target_dependencies(dbus_dr16_lib rclcpp std_msgs dbus_dr16_interface)

add_executable(dbus_dr16_node src/dbus_dr16_node.cpp)
ament_target_dependencies(dbus_dr16_node rclcpp std_msgs dbus_dr16_interface)

target_link_libraries(dbus_dr16_node dbus_dr16_lib)

install(TARGETS
  dbus_dr16_lib 
  dbus_dr16_node
  DESTINATION lib/${PROJECT_NAME})


ament_package()
