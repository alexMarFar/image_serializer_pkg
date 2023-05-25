#!/usr/bin/env python3

# Copyright (C) - All Rights Reserved
#
# Written by Alejandra Martínez Fariña <alejandra.mf23be@gmail.com>
# Licensed under the Apache License, Version 2.0


import rclpy
from rclpy.node import Node
from stereo_msgs.msg import DisparityImage
from rclpy.serialization import serialize_message

class DisparityImageSerializer(Node):
    def __init__(self):
        super().__init__('disparity_image_serializer_py')
        
        # Get the topic name parameter value
        self.topic_name = self.declare_parameter('topic_name', 'disparity_image_topic').value
        
        self.disparity_image_subscription = self.create_subscription(
            DisparityImage,
            self.topic_name,
            self.disparity_image_callback,
            10
        )
        self.disparity_image_subscription  # prevent unused variable warning

    def disparity_image_callback(self, disparity_image_msg):
         # Serialize the DisparityImage message
        serialized_msg = serialize_message(disparity_image_msg)
        msg_size = len(serialized_msg)
        print('python size: ' + str(msg_size) + ' bytes')
        pass

def main(args=None):
    rclpy.init(args=args)
    disparity_image_serializer = DisparityImageSerializer()
    rclpy.spin(disparity_image_serializer)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

