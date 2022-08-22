cmake_minimum_required(VERSION 3.5)
project(xcontroller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories("${CMAKE_CURRENT_LIST_DIR}/include/xcontroller")
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(ament_cmake REQUIRED)
find_package(message_filters REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(lgsvl_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(xcontroller src/xcontroller.cpp)
ament_target_dependencies(xcontroller message_filters rclcpp std_msgs sensor_msgs lgsvl_msgs geometry_msgs nav_msgs) 

target_link_libraries(${PROJECT_NAME} m)

install(TARGETS
  xcontroller
  DESTINATION lib/${PROJECT_NAME})

ament_package()