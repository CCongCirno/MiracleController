cmake_minimum_required(VERSION 3.8)
project(can)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(fmt REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(can_interface REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}/include)

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

add_library(can_lib src/can.cpp)
ament_target_dependencies(can_lib rclcpp std_msgs can_interface fmt)
target_link_libraries(can_lib
  fmt::fmt
)


add_executable(can0 src/can0.cpp)
ament_target_dependencies(can0 rclcpp std_msgs can_interface fmt)

add_executable(can1 src/can1.cpp)
ament_target_dependencies(can1 rclcpp std_msgs can_interface fmt)

target_link_libraries(can0
  fmt::fmt
  can_lib
)

target_link_libraries(can1
  fmt::fmt
  can_lib
)

install(TARGETS
  can0
  can1
  can_lib
  DESTINATION lib/${PROJECT_NAME})

ament_package()
