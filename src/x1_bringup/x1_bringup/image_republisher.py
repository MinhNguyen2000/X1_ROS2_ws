#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
import cv2

class ImageRepublisher(Node):
    def __init__(self):
        super().__init__('image_republisher')
        # Best Effort QoS profile
        qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)

        self.sub = self.create_subscription(
            msg_type=CompressedImage,
            topic='/camera/color/image_raw/compressed',
            callback=self.callback,
            qos_profile=qos
        )

        self.pub = self.create_publisher(
            msg_type=Image,
            topic='/camera/color/uncompressed',
            qos_profile=qos
        )
        
        self.bridge=CvBridge()
        
    def callback(self, msg:CompressedImage):
        try:
            cv_img = self.bridge.compressed_imgmsg_to_cv2(msg)
            # Republish as raw image with same QoS
            img_msg = self.bridge.cv2_to_imgmsg(cv_img, encoding='bgr8')
            img_msg.header = msg.header
            self.pub.publish(img_msg)
        except Exception as e:
            self.get_logger().error(f'Processing error: {str(e)}')

def main():
    rclpy.init()
    image_republisher = ImageRepublisher()
    rclpy.spin(image_republisher)
    rclpy.shutdown()

if __name__ == '__main__':
    main()