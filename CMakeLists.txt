cmake_minimum_required(VERSION 3.8)
project(serializer_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(stereo_msgs REQUIRED)

# Add C++ executable
add_executable(image_serializer_cpp src/image_serializer_cpp.cpp)
ament_target_dependencies(image_serializer_cpp rclcpp sensor_msgs)
add_executable(camera_info_serializer_cpp src/camera_info_serializer_cpp.cpp)
ament_target_dependencies(camera_info_serializer_cpp rclcpp sensor_msgs)
add_executable(compressed_image_serializer_cpp src/compressed_image_serializer_cpp.cpp)
ament_target_dependencies(compressed_image_serializer_cpp rclcpp sensor_msgs)
add_executable(disparity_image_serializer_cpp src/disparity_image_serializer_cpp.cpp)
ament_target_dependencies(disparity_image_serializer_cpp rclcpp stereo_msgs)
add_executable(point_cloud2_serializer_cpp src/point_cloud2_serializer_cpp.cpp)
ament_target_dependencies(point_cloud2_serializer_cpp rclcpp sensor_msgs)

# Install C++ executable
install(TARGETS
  image_serializer_cpp
  camera_info_serializer_cpp
  compressed_image_serializer_cpp
  disparity_image_serializer_cpp
  point_cloud2_serializer_cpp
  DESTINATION lib/${PROJECT_NAME})

# Add Python executable
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  scripts/image_serializer_python.py
  scripts/camera_info_serializer_python.py
  scripts/compressed_image_serializer_python.py
  scripts/disparity_image_serializer_python.py
  scripts/point_cloud2_serializer_python.py
  DESTINATION lib/${PROJECT_NAME})

# Install launch file
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

# Install package.xml
ament_package()
