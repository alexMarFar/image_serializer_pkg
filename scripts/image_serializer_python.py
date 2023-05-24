#!/usr/bin/env python3 
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.serialization import serialize_message

class ImageSerializer(Node):
    def __init__(self):
        super().__init__('image_serializer_py')
        
        # Get the topic name parameter value
        self.topic_name = self.declare_parameter('topic_name', 'image_topic').value
        
        self.image_subscription = self.create_subscription(
            Image,
            self.topic_name,
            self.image_callback,
            10
        )
        self.image_subscription  # prevent unused variable warning

    def image_callback(self, image_msg):
         # Serialize the Image message
        serialized_msg = serialize_message(image_msg)
        msg_size = len(serialized_msg)
        print('python')
        print(msg_size)
        pass

def main(args=None):
    rclpy.init(args=args)
    image_serializer = ImageSerializer()
    rclpy.spin(image_serializer)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

