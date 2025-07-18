cmake_minimum_required(VERSION 3.8)
project(ros_basic)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosbasic_msgs REQUIRED)

install(PROGRAMS
  learning_ros/publisher.py    # tao folder: lib , goi ros_publisher.py
  learning_ros/subscriber.py   # tao folder: lib , goi ros_subscriber.py
  learning_ros/scan_subscriber.py
  learning_ros/move_robot.py
  learning_ros/interface_robot.py
  learning_ros/test_code.py
  learning_ros/test_code2.py
  learning_ros/test_spin.py
  learning_ros/simple_server.py
  learning_ros/simple_client.py
  learning_ros/baitap_server.py
  learning_ros/baitap_client.py
  learning_ros/danh_server.py
  learning_ros/safety.py
  learning_ros/brake.py
  learning_ros/brake2.py
  learning_ros/visual.py
  learning_ros/visual2.py
  learning_ros/safety_visual.py
  learning_ros/eab.py
  
  
  tf/static.py
  tf/dynamic.py
  tf/auto_avoid.py
  tf/testcode.py


  DESTINATION lib/${PROJECT_NAME})   #copy 2 file ros_publisher.py, ros_subscriber.py vao lib/ros_basic

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()