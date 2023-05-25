#!/usr/bin/env python3

# Copyright (C) - All Rights Reserved
#
# Written by Alejandra Martínez Fariña <alejandra.mf23be@gmail.com>
# Licensed under the Apache License, Version 2.0


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from rclpy.serialization import serialize_message

class CompressedImageSerializer(Node):
    def __init__(self):
        super().__init__('compressed_image_serializer_py')
        
        # Get the topic name parameter value
        self.topic_name = self.declare_parameter('topic_name', 'compressed_image_topic').value
        
        self.compressed_image_subscription = self.create_subscription(
            CompressedImage,
            self.topic_name,
            self.compressed_image_callback,
            10
        )
        self.compressed_image_subscription  # prevent unused variable warning

    def compressed_image_callback(self, compressed_image_msg):
         # Serialize the CompressedImage message
        serialized_msg = serialize_message(compressed_image_msg)
        msg_size = len(serialized_msg)
        print('python size: ' + str(msg_size) + ' bytes')
        pass

def main(args=None):
    rclpy.init(args=args)
    compressed_image_serializer = CompressedImageSerializer()
    rclpy.spin(compressed_image_serializer)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

