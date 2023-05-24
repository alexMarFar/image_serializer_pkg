// Copyright (C) - All Rights Reserved
//
// Written by Alejandra Martínez Fariña <alejandra.mf23be@gmail.com>
// Licensed under the Apache License, Version 2.0

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <rclcpp/serialization.hpp>

class ImageSerializer : public rclcpp::Node
{
public:
  ImageSerializer() : Node("image_serializer_cpp")
  {
    // Get the topic name parameter value
    topic_name_ = declare_parameter<std::string>("topic_name", "image_topic");

    // Create a subscriber to the image topic
    image_serializer_ = create_subscription<sensor_msgs::msg::Image>(
      topic_name_, 10, std::bind(&ImageSerializer::imageCallback, this, std::placeholders::_1));
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr image_msg)
  {
    //Serialize the Image and CameraInfo messages
    rclcpp::SerializedMessage serialized_data_img;
    rclcpp::Serialization<sensor_msgs::msg::Image> image_serialization;
    const void* image_ptr = reinterpret_cast<const void*>(image_msg.get());
    image_serialization.serialize_message(image_ptr, &serialized_data_img);
    size_t image_msg_size = serialized_data_img.size();
    std::cout << "      cpp size:    " << image_msg_size << " bytes" << std::endl;
  
  }

  std::string topic_name_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_serializer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageSerializer>());
  rclcpp::shutdown();
  return 0;
}

