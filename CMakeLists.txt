cmake_minimum_required(VERSION 3.5)
project(yolo_innocent_counter)

find_package(ament_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(picamera2 REQUIRED)

ament_python_install_package(${PROJECT_NAME})
ament_package()
