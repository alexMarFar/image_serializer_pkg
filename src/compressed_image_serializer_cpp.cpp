// Copyright (C) - All Rights Reserved
//
// Written by Alejandra Martínez Fariña <alejandra.mf23be@gmail.com>
// Licensed under the Apache License, Version 2.0

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <rclcpp/serialization.hpp>

class CompressedImageSerializer : public rclcpp::Node
{
public:
  CompressedImageSerializer() : Node("compressed_image_serializer_cpp")
  {
    // Get the topic name parameter value
    topic_name_ = declare_parameter<std::string>("topic_name", "compressed_image_topic");

    // Create a subscriber to the compressed_image topic
    compressed_image_serializer_ = create_subscription<sensor_msgs::msg::CompressedImage>(
      topic_name_, 10, std::bind(&CompressedImageSerializer::compressed_imageCallback, this, std::placeholders::_1));
  }

private:
  void compressed_imageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr compressed_image_msg)
  {
    //Serialize the CompressedImage and CameraInfo messages
    rclcpp::SerializedMessage serialized_data_img;
    rclcpp::Serialization<sensor_msgs::msg::CompressedImage> compressed_image_serialization;
    const void* compressed_image_ptr = reinterpret_cast<const void*>(compressed_image_msg.get());
    compressed_image_serialization.serialize_message(compressed_image_ptr, &serialized_data_img);
    size_t compressed_image_msg_size = serialized_data_img.size();
    std::cout << "      cpp size:    " << compressed_image_msg_size << " bytes" << std::endl;
  
  }

  std::string topic_name_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_image_serializer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CompressedImageSerializer>());
  rclcpp::shutdown();
  return 0;
}

