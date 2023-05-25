// Copyright (C) - All Rights Reserved
//
// Written by Alejandra Martínez Fariña <alejandra.mf23be@gmail.com>
// Licensed under the Apache License, Version 2.0

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rclcpp/serialization.hpp>

class PointCloud2Serializer : public rclcpp::Node
{
public:
  PointCloud2Serializer() : Node("point_cloud2_serializer_cpp")
  {
    // Get the topic name parameter value
    topic_name_ = declare_parameter<std::string>("topic_name", "point_cloud2_topic");

    // Create a subscriber to the point_cloud2 topic
    point_cloud2_serializer_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      topic_name_, 10, std::bind(&PointCloud2Serializer::point_cloud2Callback, this, std::placeholders::_1));
  }

private:
  void point_cloud2Callback(const sensor_msgs::msg::PointCloud2::SharedPtr point_cloud2_msg)
  {
    //Serialize the PointCloud2 and CameraInfo messages
    rclcpp::SerializedMessage serialized_data_img;
    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> point_cloud2_serialization;
    const void* point_cloud2_ptr = reinterpret_cast<const void*>(point_cloud2_msg.get());
    point_cloud2_serialization.serialize_message(point_cloud2_ptr, &serialized_data_img);
    size_t point_cloud2_msg_size = serialized_data_img.size();
    std::cout << "      cpp size:    " << point_cloud2_msg_size << " bytes" << std::endl;
  
  }

  std::string topic_name_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud2_serializer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloud2Serializer>());
  rclcpp::shutdown();
  return 0;
}

