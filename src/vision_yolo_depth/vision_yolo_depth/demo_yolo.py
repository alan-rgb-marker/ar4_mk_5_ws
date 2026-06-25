#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
from ultralytics import YOLO


class YoloDetector(Node):

    def __init__(self):
        super().__init__('yolo_detector')

        self.bridge = CvBridge()

        # 載入模型
        self.model = YOLO("/home/alan/Moveit2/ar4_mk_5_ws/src/vision_yolo_depth/yolo/real_shelf_best.pt")

        # 訂閱相機
        self.image_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )

        self.get_logger().info("YOLO Detector Started")

    def image_callback(self, msg):

        try:
            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )
        except Exception as e:
            self.get_logger().error(str(e))
            return

        # YOLO推論
        results = self.model(frame, verbose=False)

        annotated_frame = results[0].plot()

        # 顯示
        cv2.imshow("YOLO Detection", annotated_frame)
        # cv2.imshow("YOLO Detection", frame)
        cv2.waitKey(1)


def main(args=None):

    rclpy.init(args=args)

    node = YoloDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    cv2.destroyAllWindows()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()