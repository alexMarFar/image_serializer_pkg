#!/usr/bin/env python3

# Copyright (C) - All Rights Reserved
#
# Written by Alejandra Martínez Fariña <alejandra.mf23be@gmail.com>
# Licensed under the Apache License, Version 2.0


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from rclpy.serialization import serialize_message

class CameraInfoSerializer(Node):
    def __init__(self):
        super().__init__('camera_info_serializer_py')
        
        # Get the topic name parameter value
        self.topic_name = self.declare_parameter('topic_name', 'camera_info_topic').value
        
        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            self.topic_name,
            self.camera_info_callback,
            10
        )
        self.camera_info_subscription  # prevent unused variable warning

    def camera_info_callback(self, camera_info_msg):
         # Serialize the CameraInfo message
        serialized_msg = serialize_message(camera_info_msg)
        msg_size = len(serialized_msg)
        print('python size: ' + str(msg_size) + ' bytes')
        pass

def main(args=None):
    rclpy.init(args=args)
    camera_info_serializer = CameraInfoSerializer()
    rclpy.spin(camera_info_serializer)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

