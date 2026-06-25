#!/usr/bin/env python3

import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class DepthCenterViewer(Node):

    def __init__(self):
        super().__init__('depth_center_viewer')

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/camera/ir/image_raw',
            self.depth_callback,
            10
        )

        self.get_logger().info("Depth Center Viewer Started")

    def depth_callback(self, msg):

        depth_image = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='passthrough'
        )

        h, w = depth_image.shape[:2]

        center_x = w // 2
        center_y = h // 2

        depth_value = depth_image[center_y, center_x]

        if msg.encoding == "16UC1" or msg.encoding == "mono16":
            distance_m = float(depth_value) / 1000.0

        elif msg.encoding == "32FC1":
            distance_m = float(depth_value)

        else:
            self.get_logger().warn(
                f"Unsupported encoding: {msg.encoding}"
            )
            return

        # ==========================
        # 深度圖轉可視化影像
        # ==========================
        depth_vis = cv2.normalize(
            depth_image,
            None,
            0,
            255,
            cv2.NORM_MINMAX
        )

        depth_vis = depth_vis.astype(np.uint8)

        depth_vis = cv2.cvtColor(
            depth_vis,
            cv2.COLOR_GRAY2BGR
        )

        # ==========================
        # 畫中心紅點
        # ==========================
        cv2.circle(
            depth_vis,
            (center_x, center_y),
            5,
            (0, 0, 255),
            -1
        )

        # ==========================
        # 顯示距離文字
        # ==========================
        cv2.putText(
            depth_vis,
            f"{distance_m:.3f} m",
            (center_x + 10, center_y - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 0, 255),
            2
        )

        cv2.imshow("Depth Image", depth_vis)
        cv2.waitKey(1)

        self.get_logger().info(
            f"Center Distance = {distance_m:.3f} m"
        )


def main(args=None):

    rclpy.init(args=args)

    node = DepthCenterViewer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    cv2.destroyAllWindows()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()