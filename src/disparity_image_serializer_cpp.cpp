// Copyright (C) - All Rights Reserved
//
// Written by Alejandra Martínez Fariña <alejandra.mf23be@gmail.com>
// Licensed under the Apache License, Version 2.0

#include <rclcpp/rclcpp.hpp>
#include <stereo_msgs/msg/disparity_image.hpp>
#include <rclcpp/serialization.hpp>

class DisparityImageSerializer : public rclcpp::Node
{
public:
  DisparityImageSerializer() : Node("disparity_image_serializer_cpp")
  {
    // Get the topic name parameter value
    topic_name_ = declare_parameter<std::string>("topic_name", "disparity_image_topic");

    // Create a subscriber to the disparity_image topic
    disparity_image_serializer_ = create_subscription<stereo_msgs::msg::DisparityImage>(
      topic_name_, 10, std::bind(&DisparityImageSerializer::disparity_imageCallback, this, std::placeholders::_1));
  }

private:
  void disparity_imageCallback(const stereo_msgs::msg::DisparityImage::SharedPtr disparity_image_msg)
  {
    //Serialize the DisparityImage and CameraInfo messages
    rclcpp::SerializedMessage serialized_data_img;
    rclcpp::Serialization<stereo_msgs::msg::DisparityImage> disparity_image_serialization;
    const void* disparity_image_ptr = reinterpret_cast<const void*>(disparity_image_msg.get());
    disparity_image_serialization.serialize_message(disparity_image_ptr, &serialized_data_img);
    size_t disparity_image_msg_size = serialized_data_img.size();
    std::cout << "      cpp size:    " << disparity_image_msg_size << " bytes" << std::endl;
  
  }

  std::string topic_name_;
  rclcpp::Subscription<stereo_msgs::msg::DisparityImage>::SharedPtr disparity_image_serializer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DisparityImageSerializer>());
  rclcpp::shutdown();
  return 0;
}

