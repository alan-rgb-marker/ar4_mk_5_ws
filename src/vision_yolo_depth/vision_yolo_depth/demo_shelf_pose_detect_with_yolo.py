import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener, TransformException
import tf2_geometry_msgs  # 提供 do_transform_point 功能

import cv2
import numpy as np
from cv_bridge import CvBridge
from ultralytics import YOLO

class YoloPoseTfDemo(Node):
    def __init__(self):
        super().__init__('yolo_pose_tf_demo')

        # 1. 初始化工具與 YOLO-Pose 模型
        self.bridge = CvBridge()
        # 註：請換成你自己訓練的 YOLO-Pose 模型路徑
        self.model = YOLO('/home/alan/Moveit2/ar4_mk_5_ws/src/vision_yolo_depth/yolo/real_shelf_pose_best.pt')

        # 2. TF2 初始化 (用於將相機座標轉換至 base_link)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.target_frame = 'base_link'

        # 3. 變數快取
        self.depth_image = None
        self.camera_info = None

        # 4. 訂閱者 (確保 topic 名稱與你的硬體一致)
        self.info_sub = self.create_subscription(CameraInfo, '/camera/color/camera_info', self.info_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/camera/depth/image_raw', self.depth_callback, 10)
        self.rgb_sub = self.create_subscription(Image, '/camera/color/image_raw', self.rgb_callback, 10)

        self.get_logger().info("【系統】YOLO-Pose 座標轉換節點已啟動！等待影像...")

    def info_callback(self, msg):
        """ 取得相機內參與相機的 frame_id """
        if self.camera_info is None:
            self.camera_info = msg
            self.get_logger().info(f"【資訊】相機內參載入成功！Frame: {msg.header.frame_id}")

    def depth_callback(self, msg):
        """ 快取深度圖並轉為公尺 (m) """
        raw_depth = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
        if msg.encoding == '16UC1':
            self.depth_image = raw_depth.astype(np.float32) / 1000.0
        else:
            self.depth_image = raw_depth.astype(np.float32)

    def rgb_callback(self, msg):
        """ 執行 YOLO-Pose 並進行 3D 座標轉換 """
        # 確保深度圖和相機資訊已經準備好
        if self.camera_info is None or self.depth_image is None:
            return

        rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        annotated_frame = rgb_image.copy()
        
        # 執行 YOLO-Pose 推理
        results = self.model(rgb_image, stream=True, conf=0.5, verbose=False)

        for r in results:
            annotated_frame = r.plot()  # 畫出預設的骨架與框
            
            # 確保有偵測到關鍵點 (Pose)
            if r.keypoints is not None and len(r.keypoints.data) > 0:
                
                # 這裡假設抓取畫面上第一個物件的「第 0 個」關鍵點作為 Demo
                # 資料結構通常是: [物件數量, 關鍵點數量, 3 (x, y, conf)]
                keypoints = r.keypoints.data[0].cpu().numpy()
                target_kp = keypoints[0]  # 取得你想要的特定點 (依你的模型定義更改 index)
                
                px, py, conf = target_kp

                # 信心度大於閾值才採算
                if conf > 0.5:
                    u, v = int(px), int(py)
                    cv2.circle(annotated_frame, (u, v), 20, (0, 0, 255), -1) # 畫紅點標示

                    # --- 步驟 A: 從 2D 像素推算 3D 相機座標 ---
                    # 避免超出影像邊界
                    if 0 <= v < self.depth_image.shape[0] and 0 <= u < self.depth_image.shape[1]:
                        # z = self.depth_image[v, u]
                        x = self.depth_image[v, u]
                        
                        if x > 0.0: # 深度不能為 0
                            fx = self.camera_info.k[0]
                            fy = self.camera_info.k[4]
                            cx = self.camera_info.k[2]
                            cy = self.camera_info.k[5]

                            # 根據相機針孔模型公式 (Deprojection)
                            y = -1 * (u - cx) * x / fx
                            z = -1 * (v - cy) * x / fy

                            # --- 步驟 B: 建立 PointStamped 準備 TF 轉換 ---
                            camera_point = PointStamped()
                            camera_point.header.frame_id = self.camera_info.header.frame_id
                            camera_point.header.stamp = msg.header.stamp
                            camera_point.point.x = float(x)
                            camera_point.point.y = float(y)
                            camera_point.point.z = float(z)

                            # --- 步驟 C: 轉換至 base_link 座標系 ---
                            try:
                                # 查詢 base_link 到 相機 frame 的變換矩陣
                                transform = self.tf_buffer.lookup_transform(
                                    self.target_frame,
                                    camera_point.header.frame_id,
                                    rclpy.time.Time()
                                )
                                
                                # 執行轉換
                                base_link_point = tf2_geometry_msgs.do_transform_point(camera_point, transform)
                                
                                # 顯示最終要的 base_link 座標
                                self.get_logger().info(
                                    f"抓取點在 {self.target_frame} 的座標 -> "
                                    f"X: {base_link_point.point.x:.3f}, "
                                    f"Y: {base_link_point.point.y:.3f}, "
                                    f"Z: {base_link_point.point.z:.3f}"
                                )

                            except TransformException as ex:
                                self.get_logger().warning(f"TF 轉換失敗: {ex}")
                        else:
                            self.get_logger().warning("該點深度值為 0，可能反光或超出測距範圍。")

        cv2.imshow("YOLO-Pose Base Link Demo", annotated_frame)
        # cv2.imshow("YOLO-Pose Base Link Demo", rgb_image)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = YoloPoseTfDemo()
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