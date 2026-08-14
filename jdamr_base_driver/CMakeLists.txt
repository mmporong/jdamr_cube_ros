cmake_minimum_required(VERSION 3.16)
project(jdamr_base_driver)

# colcon 기본은 빌드타입 미지정 → 최적화 없음. 성능 경로가 있는 노드라 Release 기본.
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(serial_port src/serial_port.cpp)
target_include_directories(serial_port PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(serial_port PUBLIC cxx_std_17)

add_executable(base_driver_node src/base_driver_node.cpp)
target_link_libraries(base_driver_node serial_port)
ament_target_dependencies(base_driver_node
  rclcpp nav_msgs sensor_msgs geometry_msgs tf2 tf2_ros)

install(TARGETS base_driver_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_protocol test/test_protocol.cpp)
  target_include_directories(test_protocol PRIVATE include)
  target_compile_features(test_protocol PRIVATE cxx_std_17)
endif()

ament_package()
