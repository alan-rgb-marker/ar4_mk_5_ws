import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo

import cv2
import numpy as np
from cv_bridge import CvBridge


class CameraReceiverNode(Node):
    def __init__(self):
        super().__init__('camera_receiver_node')

        self.bridge = CvBridge()
        self.depth_image = None
        self.camera_info = None

        self.info_sub = self.create_subscription(
            CameraInfo,
            '/camera/color/camera_info',
            self.info_callback,
            10
        )
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_callback,
            10
        )
        self.rgb_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.rgb_callback,
            10
        )

        self.get_logger().info("相機接收節點已啟動，等待影像...")

    def info_callback(self, msg: CameraInfo):
        if self.camera_info is None:
            self.camera_info = msg
            self.get_logger().info(f"相機內參載入成功！Frame: {msg.header.frame_id}")

    def depth_callback(self, msg: Image):
        raw = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
        if msg.encoding == '16UC1':
            self.depth_image = raw.astype(np.float32) / 1000.0
        else:
            self.depth_image = raw.astype(np.float32)

    def rgb_callback(self, msg: Image):
        if self.camera_info is None or self.depth_image is None:
            return

        rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        cv2.imshow("RGB", rgb_image)
        cv2.waitKey(1)


def main():
    rclpy.init()
    node = CameraReceiverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()