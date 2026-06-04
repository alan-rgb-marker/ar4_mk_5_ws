import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, Pose
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import Buffer, TransformListener, TransformException

import cv2
import numpy as np
import open3d as o3d
from cv_bridge import CvBridge
from ultralytics import YOLO
from scipy.spatial.transform import Rotation as R
from vision_interfaces.srv import ShelfCoodinate

class ShelfPoseDetector(Node):

    def __init__(self):
        super().__init__('shelf_pose_detector')
        
        # ==========================================
        # 1. 影像與相機資訊訂閱 (Depth / RGB / Info)
        # ==========================================
        self.depth_sub = self.create_subscription(
            Image, '/depth_camera/depth_image', self.depth_camera_callback, 5
        )
        self.camera_sub = self.create_subscription(
            Image, '/depth_camera/image', self.camera_callback, 5
        )
        self.info_sub = self.create_subscription(
            CameraInfo, '/depth_camera/camera_info', self.camera_info_callback, 5
        )

        # 工具與變數初始化
        self.depth_bridge = CvBridge()
        self.camera_bridge = CvBridge()
        self.depth_image = None
        
        self.k_received = False
        self.cx, self.cy, self.fx, self.fy = 0.0, 0.0, 0.0, 0.0
        
        # ==========================================
        # 2. YOLO 物體偵測模型
        # ==========================================
        self.model = YOLO(
            '/home/alan/Moveit2/ar4_mk_5_ws/src/vision_yolo_depth/yolo/new_gz_wheel_shelf.pt'
        )
        self.bbox = None
        self.yolo_results = None
        
        # ==========================================
        # 3. CAD 模型與 Open3D 點雲初始化 (ICP 用)
        # ==========================================
        self.mesh = o3d.io.read_triangle_mesh(
            "/home/alan/Moveit2/ar4_mk_5_ws/src/vision_yolo_depth/model/wheel-holder-no.stl"
        )
        self.mesh.scale(0.001, center=(0, 0, 0)) # mm 轉 m
        self.mesh.compute_vertex_normals()
        self.source_pcd = self.mesh.sample_points_uniformly(number_of_points=15000)
        
        # ==========================================
        # 4. TF2 坐標變換監聽器
        # ==========================================
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.camera_to_base_transform = None  # 儲存最新的 TF 矩陣
        self.tf_timer = self.create_timer(0.5, self.tf_callback) # 定期更新 TF 快取
        
        # 最終儲存轉換到 base_link 後的架子座標
        self.goal_shelf_pose_base = None 
        
        # ==========================================
        # 5. 提供給 MoveIt 的 ROS2 Service
        # ==========================================
        self.shelf_pose_service = self.create_service(
            ShelfCoodinate, 'shelf_coord', self.shelf_pose_callback
        )

    def camera_info_callback(self, msg):
        """ 解析相機內參 """
        if not self.k_received:
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.k_received = True

    def camera_callback(self, msg):
        """ YOLO 影像偵測：獲取架子的 2D Bounding Box """
        if self.depth_image is None or not self.k_received:
            return

        frame = self.camera_bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.yolo_results = self.model.predict(frame, conf=0.80, verbose=False)
                
        if len(self.yolo_results[0].boxes) == 0:
            self.yolo_results = None
            return

        boxes_data = self.yolo_results[0].boxes.data.cpu().numpy()
        
        # 寬度安全過濾 (排除過小的誤檢)
        h, w = self.depth_image.shape[:2]
        y_max_idx = max(0, min(int(boxes_data[0][3]), h - 1))
        x_max_idx = max(0, min(int(boxes_data[0][2]), w - 1))
        x_min_idx = max(0, min(int(boxes_data[0][0]), w - 1))
        
        xmax_z = self.depth_image[y_max_idx, x_max_idx] 
        xmin_z = self.depth_image[y_max_idx, x_min_idx]
        real_width = (boxes_data[0][2] * xmax_z / self.fx) - (boxes_data[0][0] * xmin_z / self.fx)
        
        if real_width < 0.111:
            return

        # 記錄有效的 2D 框位置
        self.bbox = [int(boxes_data[0][0]), int(boxes_data[0][1]), int(boxes_data[0][2]), int(boxes_data[0][3])]

    def depth_camera_callback(self, msg):
        """ 核心演算法：ROI 轉點雲 -> ICP 配準 -> 找出架子 Tip 點 (相機坐標系) """
        self.depth_image = self.depth_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        
        if self.bbox is None or self.yolo_results is None or not self.k_received:
            return
        
        # 1. 切出 YOLO 的 ROI 範圍
        xmin, ymin, xmax, ymax = self.bbox
        roi = self.depth_image[ymin:ymax, xmin:xmax].astype(np.float32)
        h, w = roi.shape

        # 2. 建立像素網格並計算相機坐標系 (OpenCV 標準: X右、Y下、Z前)
        u, v = np.meshgrid(np.arange(w), np.arange(h))
        z_opt = roi
        mask = (z_opt > 0.01) & (z_opt < 1.5) # 濾除無效深度與背景

        x_opt = (u - self.cx + xmin) * z_opt / self.fx
        y_opt = (v - self.cy + ymin) * z_opt / self.fy

        # 3. 轉為 ROS 影像坐標系標準 (X前、Y左、Z上)
        points = np.stack((z_opt[mask], -x_opt[mask], -y_opt[mask]), axis=-1)
        if len(points) == 0:
            return
        
        # 4. Open3D 點雲初始化
        target_pcd = o3d.geometry.PointCloud()
        target_pcd.points = o3d.utility.Vector3dVector(points)
        centroid = np.mean(points, axis=0)
        
        # 5. ICP 初始位姿估計
        init_trans = np.eye(4)
        init_trans[:3, 3] = [centroid[0] + 0.04, centroid[1], centroid[2]] # 稍微後推防邊緣扯動
        init_inv = np.linalg.inv(init_trans)
        
        # 6. 執行 ICP 配準 (點到面 Fine Alignment)
        icp_result = o3d.pipelines.registration.registration_icp(
            target_pcd, self.source_pcd, 0.05, init_inv,
            o3d.pipelines.registration.TransformationEstimationPointToPlane()
        )
        
        # 取得從 CAD 到相機坐標系的轉換矩陣 T
        T = np.linalg.inv(icp_result.transformation)
        
        # 7. 計算架子尖端 (Tip) 在相機坐標系的位置
        tip_local = np.array([0.031485, 0.000, 0.105222, 1.0])
        tip_camera = T @ tip_local
        
        # 8. 封裝成臨時的 PoseStamped 訊息 (準備給 tf2 轉換)
        self.shelf_pose_camera = PoseStamped()
        self.shelf_pose_camera.header.frame_id = "depth_camera"
        self.shelf_pose_camera.pose.position.x = float(tip_camera[0])
        self.shelf_pose_camera.pose.position.y = float(tip_camera[1])
        self.shelf_pose_camera.pose.position.z = float(tip_camera[2])
        
        q = R.from_matrix(T[:3, :3]).as_quat()
        self.shelf_pose_camera.pose.orientation.x = float(q[0])
        self.shelf_pose_camera.pose.orientation.y = float(q[1])
        self.shelf_pose_camera.pose.orientation.z = float(q[2])
        self.shelf_pose_camera.pose.orientation.w = float(q[3])

    def tf_callback(self):
        """ 監聽並將相機下的目標坐標轉換到基座坐標系 (base_link) """
        if self.yolo_results is None or not hasattr(self, 'shelf_pose_camera'):
            return
            
        try:
            # 尋找從 depth_camera 到 base_link 的當前 TF 轉換
            self.camera_to_base_transform = self.tf_buffer.lookup_transform(
                'base_link', 'depth_camera', rclpy.time.Time()
            )
            # 運用 tf2 工具直接把相機坐標系的 Pose 轉到 base_link 下
            self.goal_shelf_pose_base = do_transform_pose(
                self.shelf_pose_camera.pose, self.camera_to_base_transform
            )
            self.get_logger().info(f"成功更新 base_link 坐標: X={self.goal_shelf_pose_base}")
        except TransformException as e:
            self.get_logger().error(f'TF 坐標轉換失敗: {e}')

    def shelf_pose_callback(self, request, response):
        """ 機械手臂請求架子坐標時的 Service 回應 """
        # 確保資料已被偵測且已完成 TF 轉換
        while self.yolo_results is None or self.goal_shelf_pose_base is None:
            rclpy.spin_once(self, timeout_sec=0.05)
            
        if request.req_cmd != "get_shelf_cood":
            response.status_message = "Invalid request command."
            return response

        # 填入計算完並轉到 base_link 的位置
        response.shelf_pose = self.goal_shelf_pose_base
        
        # 覆寫為固定的機械手臂夾取姿態 (沿用原邏輯)
        response.shelf_pose.orientation.x = 0.704
        response.shelf_pose.orientation.y = 0.704
        response.shelf_pose.orientation.z = 0.062
        response.shelf_pose.orientation.w = 0.062
        
        response.shelf_vel = 0.0 # 移除速度檢測，帶入預設 0
        response.status_message = "success"

        self.get_logger().info(f"發送架子夾取目標點至 MoveIt: {response.shelf_pose}\n")
        return response

def main():
    rclpy.init()
    node = ShelfPoseDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()