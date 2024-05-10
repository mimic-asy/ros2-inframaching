cmake_minimum_required(VERSION 3.8)
project(my_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED COMPONENTS sync_policies)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core highgui videoio features2d)
find_package(rclcpp_components REQUIRED)

add_executable(infra1_pass src/infra1_pass.cpp)
ament_target_dependencies(infra1_pass rclcpp sensor_msgs cv_bridge OpenCV rclcpp_components message_filters)

install(TARGETS infra1_pass
    DESTINATION lib/${PROJECT_NAME}
)

add_executable(image_subscriber src/image_subscriber.cpp)
ament_target_dependencies(image_subscriber rclcpp sensor_msgs cv_bridge OpenCV rclcpp_components message_filters)

install(TARGETS image_subscriber
    DESTINATION lib/${PROJECT_NAME}
)

add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

install(TARGETS talker
  DESTINATION lib/${PROJECT_NAME})

add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

install(TARGETS listener
  DESTINATION lib/${PROJECT_NAME})

add_executable(my_node src/my_node.cpp)
target_include_directories(my_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(my_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS my_node
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

ament_package()
