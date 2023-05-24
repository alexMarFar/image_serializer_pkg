cmake_minimum_required(VERSION 3.8)
project(image_serializer_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Add C++ executable
add_executable(image_serializer_cpp src/image_serializer_cpp.cpp)
ament_target_dependencies(image_serializer_cpp rclcpp sensor_msgs)

# Install C++ executable
install(TARGETS
  image_serializer_cpp
  DESTINATION lib/${PROJECT_NAME})

# Add Python executable
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  scripts/image_serializer_python.py
  DESTINATION lib/${PROJECT_NAME})

# Install launch file
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

# Install package.xml
ament_package()
