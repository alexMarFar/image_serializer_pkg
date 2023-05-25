// Copyright (C) - All Rights Reserved
//
// Written by Alejandra Martínez Fariña <alejandra.mf23be@gmail.com>
// Licensed under the Apache License, Version 2.0

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <rclcpp/serialization.hpp>

class CameraInfoSerializer : public rclcpp::Node
{
public:
  CameraInfoSerializer() : Node("camera_info_serializer_cpp")
  {
    // Get the topic name parameter value
    topic_name_ = declare_parameter<std::string>("topic_name", "camera_info_topic");

    // Create a subscriber to the camera_info topic
    camera_info_serializer_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      topic_name_, 10, std::bind(&CameraInfoSerializer::camera_infoCallback, this, std::placeholders::_1));
  }

private:
  void camera_infoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr camera_info_msg)
  {
    //Serialize the CameraInfo and CameraInfo messages
    rclcpp::SerializedMessage serialized_data_img;
    rclcpp::Serialization<sensor_msgs::msg::CameraInfo> camera_info_serialization;
    const void* camera_info_ptr = reinterpret_cast<const void*>(camera_info_msg.get());
    camera_info_serialization.serialize_message(camera_info_ptr, &serialized_data_img);
    size_t camera_info_msg_size = serialized_data_img.size();
    std::cout << "      cpp size:    " << camera_info_msg_size << " bytes" << std::endl;
  
  }

  std::string topic_name_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_serializer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraInfoSerializer>());
  rclcpp::shutdown();
  return 0;
}

