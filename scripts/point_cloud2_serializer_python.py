#!/usr/bin/env python3

# Copyright (C) - All Rights Reserved
#
# Written by Alejandra Martínez Fariña <alejandra.mf23be@gmail.com>
# Licensed under the Apache License, Version 2.0


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from rclpy.serialization import serialize_message

class PointCloud2Serializer(Node):
    def __init__(self):
        super().__init__('point_cloud2_serializer_py')
        
        # Get the topic name parameter value
        self.topic_name = self.declare_parameter('topic_name', 'point_cloud2_topic').value
        
        self.point_cloud2_subscription = self.create_subscription(
            PointCloud2,
            self.topic_name,
            self.point_cloud2_callback,
            10
        )
        self.point_cloud2_subscription  # prevent unused variable warning

    def point_cloud2_callback(self, point_cloud2_msg):
         # Serialize the PointCloud2 message
        serialized_msg = serialize_message(point_cloud2_msg)
        msg_size = len(serialized_msg)
        print('python size: ' + str(msg_size) + ' bytes')
        pass

def main(args=None):
    rclpy.init(args=args)
    point_cloud2_serializer = PointCloud2Serializer()
    rclpy.spin(point_cloud2_serializer)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

