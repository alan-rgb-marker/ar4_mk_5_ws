#!/usr/bin/env python3
"""
wheel_detector_node.py

功能：
  1. 訂閱彩色影像，用 YOLO 偵測 wheel/tire
  2. 訂閱深度影像，取 bounding box 中心區域平均深度
  3. 用 camera_info 做反投影，得到 camera frame 的 3D 座標
  4. 用 TF2 轉換到 base_link
  5. Publish geometry_msgs/PoseStamped
  6. 在本地端彈出視窗顯示影像與偵測座標 (確保顯示)

Topics (SUB):
  /camera/color/image_raw       sensor_msgs/Image
  /camera/depth/image_raw       sensor_msgs/Image
  /camera/color/camera_info     sensor_msgs/CameraInfo

Topics (PUB):
  /wheel/pose                   geometry_msgs/PoseStamped
  /wheel/detection_image        sensor_msgs/Image   (視覺化用，可選)
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

import numpy as np
import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, Point, Quaternion
import message_filters

import tf2_ros
import tf2_geometry_msgs  # noqa: F401  必須 import 才能讓 do_transform_pose 正常運作

from ultralytics import YOLO


class WheelDetectorNode(Node):

    def __init__(self):
        super().__init__('wheel_detector_node')

        # ── 參數 ──────────────────────────────────────────────
        # self.declare_parameter('model_path', '/home/alan/Moveit2/ar4_mk_5_ws/src/vision_yolo_depth/yolo/real_wheel_best.pt')
        self.declare_parameter('model_path', '/home/alan/Moveit2/ar4_mk_5_ws/src/vision_yolo_depth/yolo/real_shelf_pose_best.pt')
        self.declare_parameter('confidence', 0.5)
        self.declare_parameter('target_class', 'wheel')   # YOLO 類別名稱
        self.declare_parameter('center_roi_ratio', 0.3)   # 中心區域佔 bbox 的比例
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_debug_image', True)

        model_path       = self.get_parameter('model_path').value
        self.conf_thres  = self.get_parameter('confidence').value
        self.target_cls  = self.get_parameter('target_class').value
        self.roi_ratio   = self.get_parameter('center_roi_ratio').value
        self.cam_frame   = self.get_parameter('camera_frame').value
        self.base_frame  = self.get_parameter('base_frame').value
        self.pub_debug   = self.get_parameter('publish_debug_image').value

        # ── YOLO 模型 ─────────────────────────────────────────
        self.get_logger().info(f'Loading YOLO model: {model_path}')
        self.model = YOLO(model_path)
        self.get_logger().info('YOLO model loaded.')

        # ── Camera Intrinsics (等 camera_info 進來再填) ────────
        self.fx = self.fy = self.cx = self.cy = None

        # ── TF2 ───────────────────────────────────────────────
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── CV Bridge ─────────────────────────────────────────
        self.bridge = CvBridge()

        # ── Publishers ────────────────────────────────────────
        self.pose_pub = self.create_publisher(PoseStamped, '/wheel/pose', 10)
        if self.pub_debug:
            self.debug_pub = self.create_publisher(Image, '/wheel/detection_image', 10)

        # ── Subscribers (time-sync 彩色 + 深度) ───────────────
        color_sub = message_filters.Subscriber(
            self, Image, '/camera/color/image_raw')
        depth_sub = message_filters.Subscriber(
            self, Image, '/camera/depth/image_raw')

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [color_sub, depth_sub], queue_size=10, slop=0.05)
        self.ts.registerCallback(self.image_callback)

        # camera_info 只需要拿一次
        self.info_sub = self.create_subscription(
            CameraInfo, '/camera/color/camera_info',
            self.camera_info_callback, 10)

        # 建立 OpenCV 視窗
        cv2.namedWindow("Wheel Detection", cv2.WINDOW_NORMAL)

        self.get_logger().info('WheelDetectorNode ready. OpenCV Window "Wheel Detection" should be visible.')

    # ── Camera Info ───────────────────────────────────────────
    def camera_info_callback(self, msg: CameraInfo):
        if self.fx is not None:
            return  # 只需要一次
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        self.get_logger().info(
            f'Camera intrinsics: fx={self.fx:.2f} fy={self.fy:.2f} '
            f'cx={self.cx:.2f} cy={self.cy:.2f}')
        self.destroy_subscription(self.info_sub)  # 拿到後取消訂閱

    # ── 主要 Callback ─────────────────────────────────────────
    def image_callback(self, color_msg: Image, depth_msg: Image):
        if self.fx is None:
            self.get_logger().warn('Camera intrinsics not yet received, skipping frame.')
            return

        # 1. 轉換影像
        color_img = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
        depth_img = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        # RealSense 深度單位為 mm，轉成 m
        if depth_img.dtype == np.uint16:
            depth_img = depth_img.astype(np.float32) / 1000.0

        # 用於顯示的影像副本
        display_img = color_img.copy()

        # 2. YOLO 推理
        results = self.model(color_img, conf=self.conf_thres, verbose=False)

        best_box  = None
        best_conf = -1.0

        for result in results:
            for box in result.boxes:
                cls_name = self.model.names[int(box.cls)]
                if cls_name.lower() != self.target_cls.lower():
                    continue
                conf = float(box.conf)
                if conf > best_conf:
                    best_conf = conf
                    best_box  = box.xyxy[0].cpu().numpy()  # [x1,y1,x2,y2]

        if best_box is None:
            self.get_logger().info('No wheel detected.')
            self._update_display(display_img)
            if self.pub_debug:
                self._publish_ros_debug(display_img, color_msg.header)
            return  # 沒偵測到 wheel

        x1, y1, x2, y2 = best_box.astype(int)

        # 3. 取中心 ROI 的平均深度
        z = self._get_roi_depth(depth_img, x1, y1, x2, y2)
        if z is None or z <= 0.0:
            self.get_logger().warn('Invalid depth in ROI, skipping.')
            cv2.rectangle(display_img, (x1, y1), (x2, y2), (0, 0, 255), 2)  # 畫紅框表示深度無效
            self._update_display(display_img)
            if self.pub_debug:
                self._publish_ros_debug(display_img, color_msg.header)
            return

        # 4. 反投影：pixel → camera 3D
        cx_px = (x1 + x2) / 2.0
        cy_px = (y1 + y2) / 2.0
        x_cam = (cx_px - self.cx) * z / self.fx
        y_cam = (cy_px - self.cy) * z / self.fy
        z_cam = z

        # 5. 建立 PoseStamped（camera frame）
        pose_cam = PoseStamped()
        pose_cam.header.stamp    = color_msg.header.stamp
        pose_cam.header.frame_id = self.cam_frame
        pose_cam.pose.position   = Point(x=x_cam, y=y_cam, z=z_cam)
        pose_cam.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        # 6. TF2 轉換到 base_link
        pose_base = self._transform_to_base(pose_cam)
        if pose_base is None:
            self.get_logger().warn('TF transform to base_link failed.')
            cv2.rectangle(display_img, (x1, y1), (x2, y2), (255, 0, 0), 2)  # 畫藍框表示 TF 失敗
            self._update_display(display_img)
            if self.pub_debug:
                self._publish_ros_debug(display_img, color_msg.header)
            return

        # 成功轉換
        self.pose_pub.publish(pose_base)
        
        # ── 終端機印出座標 (確保印出) ──
        # 使用換行和分隔線讓輸出更明顯
        print("-" * 30)
        self.get_logger().info(
            f'\n[DETECTED] Wheel in {self.base_frame}:\n'
            f'  X: {pose_base.pose.position.x:.3f} m\n'
            f'  Y: {pose_base.pose.position.y:.3f} m\n'
            f'  Z: {pose_base.pose.position.z:.3f} m\n'
            f'  Confidence: {best_conf:.2f}'
        )
        print("-" * 30)

        # 7. 在影像上繪製結果
        # 畫綠框
        cv2.rectangle(display_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
        
        # 準備標籤
        label = (f'{self.target_cls} {best_conf:.2f} | '
                    f'XYZ: ({pose_base.pose.position.x:.2f}, '
                    f'{pose_base.pose.position.y:.2f}, '
                    f'{pose_base.pose.position.z:.2f})m')
        
        # 在框上方加上半透明黑底讓文字更清楚
        (text_width, text_height), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
        text_y = max(y1 - 10, text_height + 5)
        cv2.rectangle(display_img, (x1, text_y - text_height - 5), (x1 + text_width + 5, text_y + 5), (0, 0, 0), -1)
        
        # 畫文字
        cv2.putText(display_img, label, (x1 + 2, text_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        
        # ── 更新顯示 ──
        self._update_display(display_img)
        
        if self.pub_debug:
            self._publish_ros_debug(display_img, color_msg.header)

    # ── 工具函式 ──────────────────────────────────────────────
    def _get_roi_depth(self, depth_img, x1, y1, x2, y2):
        """取 bounding box 中心區域的平均深度（過濾無效值）"""
        h, w = depth_img.shape[:2]
        bw = x2 - x1
        bh = y2 - y1
        margin_x = int(bw * (1 - self.roi_ratio) / 2)
        margin_y = int(bh * (1 - self.roi_ratio) / 2)

        rx1 = max(x1 + margin_x, 0)
        rx2 = min(x2 - margin_x, w - 1)
        ry1 = max(y1 + margin_y, 0)
        ry2 = min(y2 - margin_y, h - 1)

        if rx2 <= rx1 or ry2 <= ry1:
            return None

        roi = depth_img[ry1:ry2, rx1:rx2]
        # 過濾深度值在合理範圍內 (例如 0.1m ~ 10m)
        valid = roi[(roi > 0.1) & (roi < 10.0)]
        if valid.size == 0:
            return None
        # 使用中位數避免極端值
        return float(np.median(valid))

    def _transform_to_base(self, pose_cam: PoseStamped):
        """用 TF2 將 PoseStamped 從 camera frame 轉到 base_link"""
        try:
            # 查找轉換
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                pose_cam.header.frame_id,
                pose_cam.header.stamp,
                timeout=Duration(seconds=0.1)
            )
            # 執行轉換
            return tf2_geometry_msgs.do_transform_pose_stamped(pose_cam, transform)
        except tf2_ros.LookupException as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
        except tf2_ros.ExtrapolationException as e:
            self.get_logger().warn(f'TF extrapolation failed: {e}')
        except Exception as e:
            self.get_logger().error(f'Unexpected TF error: {e}')
        return None

    def _update_display(self, img):
        """在本地端更新 OpenCV 視窗顯示影像"""
        try:
            cv2.imshow("Wheel Detection", img)
            cv2.waitKey(1)  # 必須加入這行，視窗才會刷新
        except Exception as e:
            self.get_logger().error(f'Error updating display: {e}')

    def _publish_ros_debug(self, img, header):
        """發布偵測結果影像到 ROS Topic"""
        try:
            msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
            msg.header = header
            self.debug_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Error publishing debug image: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = WheelDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt, shutting down.')
    finally:
        # 確保節點關閉時，OpenCV 的視窗也會自動關閉
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()