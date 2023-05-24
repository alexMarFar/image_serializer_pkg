# image_serializer_pkg
ROS2 package for image serialization in both rclpy and rclcpp

![image](https://github.com/alexMarFar/image_serializer_pkg/assets/82050691/6888882f-ad74-4f94-b4e9-99f615c05907)

## Usage
1. Clone the repository in your workspace
```bash
cd ~/ros2_ws/src
git clone https://github.com/alexMarFar/image_serializer_pkg.git
```
2. Update the topic_name in the launch file.

3. Build the package
```bash
cd ~/ros2_ws/
colcon build --packages-up-to image_serializer_pkg
source install/setup.bash
```
4. Launch the nodes
```bash
ros2 launch image_serializer_pkg image_serializer_pkg.launch.py
```
