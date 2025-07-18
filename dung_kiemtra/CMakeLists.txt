cmake_minimum_required(VERSION 3.8)
project(dung_kiemtra)

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
  dung_kiemtra/mainn.py
  dung_kiemtra/main_a.py
  dung_kiemtra/main_b.py
  dung_kiemtra/main_c.py
  # dung_kiemtra/main_d.py
  # dung_kiemtra/main_e.py


  DESTINATION lib/${PROJECT_NAME}) 

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
