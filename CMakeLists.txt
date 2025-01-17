cmake_minimum_required(VERSION 3.5)
project(gnss_comm)

set(CMAKE_BUILD_TYPE "release")
set(CMAKE_CXX_FLAGS "-std=c++11 ")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -fPIC")

# 查找ament_cmake
find_package(ament_cmake REQUIRED)
# 查找ROS 2组件
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gnss_comm_interfaces REQUIRED)

message("gnss_comm_interfaces _include_dirs: ${gnss_comm_interfaces_INCLUDE_DIRS}")
message("gnss_comm_interfaces _include_dirs: ${std_msgs_INCLUDE_DIRS}")

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3 REQUIRED)
find_package(Glog REQUIRED)
include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${GLOG_INCLUDE_DIR}
  include
)

file(GLOB_RECURSE source_files ${PROJECT_SOURCE_DIR}/src/*.cpp)
message("${source_files}")

add_library(${PROJECT_NAME} ${source_files})

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  gnss_comm_interfaces
)

target_link_libraries(${PROJECT_NAME}  ${GLOG_LIBRARIES})

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME} 
  ARCHIVE DESTINATION lib
  LIBRARY  DESTINATION lib)

ament_export_dependencies(rclcpp)
ament_export_dependencies(std_msgs)
ament_export_dependencies(gnss_comm_interfaces)
ament_export_include_directories(${INCLUDE_DIRS})
ament_export_libraries(${PROJECT_NAME})

ament_export_include_directories(include)

ament_package()