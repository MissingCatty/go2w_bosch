cmake_minimum_required(VERSION 3.5)
project(go2_imu_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(unitree_go REQUIRED)
find_package(Threads REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs imgproc)

add_executable(imu_attitude_bridge src/imu_attitude_bridge.cpp)
ament_target_dependencies(imu_attitude_bridge rclcpp sensor_msgs unitree_go)

add_executable(lidar_preview_bridge src/lidar_preview_bridge.cpp)
ament_target_dependencies(lidar_preview_bridge rclcpp sensor_msgs)

# Unitree VideoClient 返回前置摄像头 JPEG。该进程由 Web 导航模式按需启停，
# 不随建图 launch 常驻，避免与地图点云争用网络带宽。
add_library(unitree_sdk2 STATIC IMPORTED)
set_target_properties(unitree_sdk2 PROPERTIES
  IMPORTED_LOCATION "/home/unitree/unitree_sdk2/lib/aarch64/libunitree_sdk2.a"
  INTERFACE_INCLUDE_DIRECTORIES "/home/unitree/unitree_sdk2/include;/home/unitree/unitree_sdk2/thirdparty/include/ddscxx")
target_link_libraries(unitree_sdk2 INTERFACE ddsc ddscxx Threads::Threads)
add_executable(front_camera_bridge src/front_camera_bridge.cpp)
target_include_directories(front_camera_bridge PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(front_camera_bridge unitree_sdk2 ${OpenCV_LIBRARIES})

install(TARGETS imu_attitude_bridge lidar_preview_bridge front_camera_bridge
  DESTINATION lib/${PROJECT_NAME})

ament_package()
