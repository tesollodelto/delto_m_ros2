cmake_minimum_required(VERSION 3.8)
project(dg3f_b_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

# dg3f_b_tcp 라이브러리 정의 (로컬 라이브러리)
add_library(dg3f_b_tcp SHARED
  src/delto_developer_TCP.cpp
)

target_include_directories(dg3f_b_tcp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(dg3f_b_tcp
  rclcpp
)
add_library(DG_GRIPPER_HELPER SHARED IMPORTED)
set_target_properties(DG_GRIPPER_HELPER PROPERTIES
  IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/lib/libdelto_gripper_helper.so
)
# dg3f_b_driver 라이브러리 정의
add_library(dg3f_b_driver SHARED
  src/system_interface.cpp
)

target_include_directories(dg3f_b_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 로컬 라이브러리 링크 
target_link_libraries(dg3f_b_driver dg3f_b_tcp DG_GRIPPER_HELPER)

# ROS2 의존성은 ament_target_dependencies로 처리
ament_target_dependencies(dg3f_b_driver
  rclcpp
  pluginlib
  hardware_interface
)

# 설치 규칙
install(
  TARGETS dg3f_b_driver dg3f_b_tcp
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

add_executable(dg3f_b_operator_driver src/dg3f_b_operator_driver.cpp)
ament_target_dependencies(dg3f_b_operator_driver rclcpp trajectory_msgs sensor_msgs)

install(TARGETS 
  dg3f_b_operator_driver
  DESTINATION lib/${PROJECT_NAME})

  target_include_directories(dg3f_b_operator_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(
  DIRECTORY config urdf launch
  DESTINATION share/${PROJECT_NAME}
)


### test control DG3F example
add_executable(dg3f_b_test_cpp src/dg3f_b_test.cpp)
ament_target_dependencies(dg3f_b_test_cpp rclcpp trajectory_msgs)

add_executable(dg3f_b_operator_test_cpp src/dg3f_b_operator_test.cpp)
ament_target_dependencies(dg3f_b_operator_test_cpp rclcpp trajectory_msgs)

install(
  PROGRAMS 
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS 
  script/dg3f_b_test.py
  DESTINATION lib/${PROJECT_NAME}
)


pluginlib_export_plugin_description_file(hardware_interface dg3f_b_driver.xml)

install(
  FILES dg3f_b_driver.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
