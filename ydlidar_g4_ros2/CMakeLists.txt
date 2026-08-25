cmake_minimum_required(VERSION 3.16)
project(ydlidar_g4_ros2 LANGUAGES C CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Threads REQUIRED)

set(YDLIDAR_SDK_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/vendor/YDLidar-SDK)
file(GLOB YDLIDAR_SDK_SOURCES CONFIGURE_DEPENDS
  ${YDLIDAR_SDK_ROOT}/src/*.cpp
  ${YDLIDAR_SDK_ROOT}/core/base/*.cpp
  ${YDLIDAR_SDK_ROOT}/core/common/*.cpp
  ${YDLIDAR_SDK_ROOT}/core/json/*.c
  ${YDLIDAR_SDK_ROOT}/core/network/*.cpp
  ${YDLIDAR_SDK_ROOT}/core/serial/*.cpp
  ${YDLIDAR_SDK_ROOT}/core/serial/impl/unix/*.cpp
  ${YDLIDAR_SDK_ROOT}/core/serial/impl/unix/*.c)

add_library(ydlidar_sdk STATIC ${YDLIDAR_SDK_SOURCES})
target_include_directories(ydlidar_sdk SYSTEM PUBLIC
  ${YDLIDAR_SDK_ROOT}
  ${YDLIDAR_SDK_ROOT}/src
  ${YDLIDAR_SDK_ROOT}/core
  ${YDLIDAR_SDK_ROOT}/core/serial
  ${YDLIDAR_SDK_ROOT}/core/serial/impl/unix
  ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_definitions(ydlidar_sdk PRIVATE HAS_CLOCK_GETTIME=1)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(ydlidar_sdk PRIVATE -w)
endif()
target_link_libraries(ydlidar_sdk PUBLIC Threads::Threads)
set_target_properties(ydlidar_sdk PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_executable(ydlidar_g4_node src/ydlidar_g4_node.cpp)
target_include_directories(ydlidar_g4_node PRIVATE include)
target_link_libraries(ydlidar_g4_node ydlidar_sdk)
ament_target_dependencies(ydlidar_g4_node rclcpp sensor_msgs)

install(TARGETS ydlidar_g4_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY udev DESTINATION share/${PROJECT_NAME})
install(FILES vendor/YDLidar-SDK/LICENSE.txt DESTINATION share/${PROJECT_NAME}/vendor/YDLidar-SDK)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(scan_utils_test test/scan_utils_test.cpp)
  target_include_directories(scan_utils_test PRIVATE include)
endif()

ament_package()
