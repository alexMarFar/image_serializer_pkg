# serializer_pkg
ROS2 package for image serialization in both rclpy and rclcpp.

Available for:
- sensor_msgs:
  - [x] Image
  - [x] CameraInfo
- stereo_msgs:
  - [x] DisparityImage

![image](https://github.com/alexMarFar/serializer_pkg/assets/82050691/6888882f-ad74-4f94-b4e9-99f615c05907)

## Usage
1. Clone the repository in your workspace
```bash
cd ~/ros2_ws/src
git clone https://github.com/alexMarFar/serializer_pkg.git
```
2. Update the topic_name in the launch file.

3. Build the package
```bash
cd ~/ros2_ws/
colcon build --packages-up-to serializer_pkg
source install/setup.bash
```
4. Launch the nodes
- For Image (sensor_msgs)
```bash
ros2 launch serializer_pkg image.launch.py
```
- For CameraInfo (sensor_msgs)
```bash
ros2 launch serializer_pkg camera_info.launch.py
```
- For DisparityImage (stereo_msgs)
```bash
ros2 launch serializer_pkg disparity_image.launch.py
```
