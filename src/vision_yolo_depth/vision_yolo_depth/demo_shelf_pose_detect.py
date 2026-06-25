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

class ShelfDetectorDemo(Node):
    def __init__(self):
        super().__init__('shelf_detector_demo')
        
        # 1. 訂閱必備的相機資訊與影像
        self.info_sub = self.create_subscription(CameraInfo, '/camera/depth/camera_info', self.info_callback, 10)
        self.rgb_sub = self.create_subscription(Image, '/camera/color/image_raw', self.rgb_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/camera/depth/image_raw', self.depth_callback, 10)

        # 2. 初始化工具與 YOLO 模型
        self.bridge = CvBridge()
        self.model = YOLO('/home/alan/Moveit2/ar4_mk_5_ws/src/vision_yolo_depth/yolo/real_shelf_best.pt')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.yolo_detect_shelf_results = None
        self.depth_camera_to_base_rot = None
        self.goal_shelf_pose_msg = Pose()
        self.shelf_pose_msg = PoseStamped()
        
        # 3. 加載與預處理 CAD 模型 (單位：公尺)
        self.mesh = o3d.io.read_triangle_mesh("/home/alan/Moveit2/ar4_mk_5_ws/src/vision_yolo_depth/model/wheel-holder-no.stl")
        self.mesh.scale(0.001, center=(0, 0, 0)) # mm 轉 m
        self.mesh.compute_vertex_normals()
        self.source_pcd = self.mesh.sample_points_uniformly(number_of_points=15000)

        # 4. 變數快取
        self.k_received = False
        self.fx = self.fy = self.cx = self.cy = 0.0
        
        self.rgb_image = None
        self.depth_image = None
        self.bbox = None  # [xmin, ymin, xmax, ymax]
        
        # 在 __init__ 加這幾行，看看 mesh 的 bounding box
        bboxs = self.mesh.get_axis_aligned_bounding_box()
        self.get_logger().info(f"Mesh min: {bboxs.min_bound}")
        self.get_logger().info(f"Mesh max: {bboxs.max_bound}")
        center = self.mesh.get_center()
        self.get_logger().info(f"Mesh center: {center}")

    def info_callback(self, msg):
        """ 解析相機內參 """
        if not self.k_received:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.k_received = True
            self.get_logger().info("【資訊】相機內參載入成功！")

    def rgb_callback(self, msg):
        """ YOLO 2D 物件偵測 """
        if not self.k_received:
            return
        
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # 執行 YOLO 推理
        results = self.model(self.rgb_image, stream=True, conf=0.4, verbose=False)
        
        best_box = None
        best_conf = -1.0
        annotated_frame = self.rgb_image.copy()

        for r in results:
            annotated_frame = r.plot()
            if r.boxes is None or len(r.boxes) == 0:
                continue
            
            for box in r.boxes.data.cpu().numpy():
                x1, y1, x2, y2, conf, cls = box
                self.yolo_detect_shelf_results = True
                if conf > best_conf:
                    best_conf = conf
                    best_box = [int(x1), int(y1), int(x2), int(y2)]

        if best_box is not None:
            self.bbox = best_box
            # 視覺化顯示偵測結果
            cv2.rectangle(annotated_frame, (self.bbox[0], self.bbox[1]), (self.bbox[2], self.bbox[3]), (0, 255, 0), 2)
        else:
            self.bbox = None

        cv2.imshow("YOLO Detection Demo", annotated_frame)
        cv2.waitKey(1)

    def depth_callback(self, msg):
        """ 3D 點雲重建與兩階段 ICP 幾何配準 """
        if self.rgb_image is None or self.bbox is None or not self.k_received:
            return

        # 轉換深度圖並轉換為公尺 (m)
        raw_depth = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
        if msg.encoding == '16UC1':
            self.depth_image = raw_depth.astype(np.float32) / 1000.0
        else:
            self.depth_image = raw_depth.astype(np.float32)

        depth_filtered = cv2.bilateralFilter(self.depth_image.astype(np.float32), 7, 0, 25)
        # 或使用 Temporal filter (多幀平均)
        
        if self.bbox is None or self.yolo_detect_shelf_results is None or self.k_received is not True:
            return
        
        xmin, ymin, xmax, ymax = self.bbox

        # ==========================================
        # 1. 縮小 Bounding Box 範圍 (排除邊緣飛點)
        # ==========================================
        w_box = xmax - xmin
        h_box = ymax - ymin

        # 設定內縮比例 (例如 8%)
        padding_x = int(w_box * 0.08)
        padding_y = int(h_box * 0.08)

        # 更新邊界
        xmin_roi = xmin + padding_x
        xmax_roi = xmax - padding_x
        ymin_roi = ymin + padding_y
        ymax_roi = ymax - padding_y

        # 取 ROI (使用內縮後的邊界)
        roi = depth_filtered[ymin_roi:ymax_roi, xmin_roi:xmax_roi].astype(np.float32)
        h, w = roi.shape

        # ==========================================
        # 2. 建立 pixel grid (ROI 座標)
        # ==========================================
        u = np.arange(w)
        v = np.arange(h)
        uu, vv = np.meshgrid(u, v)

        # 3. 深度轉 m
        z = roi

        # 4. 有效深度 mask (排除 0 以及距離大於 1.5 公尺的背景點)
        mask = (z > 0.01) & (z < 1.5)

        # 5. 轉成 3D point cloud
        # OpenCV 光學座標: x向右, y向下, z向前
        # ★★★ 重要修改：這裡必須要用內縮後的 xmin_roi 與 ymin_roi 來做補償 ★★★
        x_opt = (uu - self.cx + xmin_roi) * z / self.fx
        y_opt = (vv - self.cy + ymin_roi) * z / self.fy
        z_opt = z

        # 轉換為 ROS 標準座標 (depth_camera frame): X向前, Y向左, Z向上
        x = z_opt
        y = -x_opt
        z = -y_opt

        points = np.stack(
            (x[mask], y[mask], z[mask]),
            axis=-1
        )

        # --- 以下繼續銜接你原本的 6. Open3D point cloud ---
        target = o3d.geometry.PointCloud()
        target.points = o3d.utility.Vector3dVector(points)
        
        # ... (後面照舊) ...
        
        # 1. 下採樣 (Voxel Filter)，讓點雲均勻化，加快 ICP 速度並減少權重失衡
        target = target.voxel_down_sample(voxel_size=0.001) # 5mm 
        # 2. 濾除孤立雜訊點 (Statistical Outlier Removal)
        target, ind = target.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
        
        target.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.02, max_nn=30))
        
        if len(points) > 0:
            # points 的形狀是 (N, 3)，取平均後會得到一組 [X_avg, Y_avg, Z_avg]
            centroid = np.mean(points, axis=0)
            x_center = centroid[0]
            y_center = centroid[1]
            z_center = centroid[2]
            # --- Debug Log ---
            # self.get_logger().info(f"目標點雲中心 (Centroid): x={x_center:.3f}, y={y_center:.3f}, z={z_center:.3f}, 總點數={len(points)}")
        else:
            # 如果沒有有效點，給予預設值 (全 0)
            x_center, y_center, z_center = 0.0, 0.0, 0.0
            self.get_logger().warning("深度圖 ROI 內沒有有效點！")
            return
        
        if self.depth_camera_to_base_rot is not None:
            # R_base_camera 是從 depth_camera 到 base_link 的旋轉
            # 因為物體在 base_link 是正的，我們需要它的反向旋轉 (相機看物體的相對旋轉)
            # Quaternion 反向即取 -x, -y, -z
            q_inv = [
                -self.depth_camera_to_base_rot.x,
                -self.depth_camera_to_base_rot.y,
                -self.depth_camera_to_base_rot.z,
                self.depth_camera_to_base_rot.w
            ]
            R_init = R.from_quat(q_inv).as_matrix()
        else:
            R_init = np.eye(3)

        init = np.eye(4)
        init[:3, :3] = R_init
        # 將 CAD 模型稍微往後推 (ROS X軸為深度方向)，確保掃描點位於模型前方，
        # 避免掃描點落入模型內部而錯誤匹配到後表面
        init[0, 3] = x_center
        init[1, 3] = y_center
        init[2, 3] = z_center
        
        # 反向 ICP (Scan 匹配到 CAD Model)
        # 解決 full-to-partial ICP 導致的 3cm 深度誤差：
        # 若將完整的 CAD 模型匹配到部分掃描點雲，會造成模型被往前拉扯約一半厚度的誤差。
        # 將 scan 匹配到 CAD，掃描點自然只會尋找 CAD 模型前表面的最近點。
        init_inv = np.linalg.inv(init)
        
        # Stage 1: Coarse alignment (Point-to-Point)
        icp_coarse = o3d.pipelines.registration.registration_icp(
            target,             # Source: partial scan
            self.source_pcd,    # Target: full CAD model (with perfect normals)
            0.3,               # 20cm threshold
            init_inv,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=30)
        )
        
        # Stage 2: Fine alignment (Point-to-Plane)
        icp_result = o3d.pipelines.registration.registration_icp(
            target,
            self.source_pcd,
            0.05,               # 5cm threshold
            icp_coarse.transformation,
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=30)
        )
        
        # 將轉換矩陣反轉回 CAD 到 Scan 的方向
        T_scan_to_cad = icp_result.transformation
        T = np.linalg.inv(T_scan_to_cad)
        # self.get_logger().info(f"ICP 變換矩陣的平移 (Origin): x={T[0,3]:.3f}, y={T[1,3]:.3f}, z={T[2,3]:.3f}")
        
        self.shelf_pose_msg.header = "camera_color_optical_frame"
        
        # 柱子尖端在模型局部空間的座標 (將 mm 轉換為公尺 m)
        tip_local = np.array([0.031485, 0.000, 0.105222, 1.0])
        
        # 將尖端座標透過 T 矩陣轉換到 depth_camera 座標系
        tip_camera = T @ tip_local
        
        self.shelf_pose_msg.pose.position.x = float(tip_camera[0])
        self.shelf_pose_msg.pose.position.y = float(tip_camera[1])
        self.shelf_pose_msg.pose.position.z = float(tip_camera[2])
        
        rot = R.from_matrix(T[:3, :3])
        q = rot.as_quat()   # (x, y, z, w)

        self.shelf_pose_msg.pose.orientation.x = float(q[0])
        self.shelf_pose_msg.pose.orientation.y = float(q[1])
        self.shelf_pose_msg.pose.orientation.z = float(q[2])
        self.shelf_pose_msg.pose.orientation.w = float(q[3])
        
        # self.get_logger().info(f"Shelf Pose in depth_camera frame: {self.shelf_pose_msg}\n")
        self.get_logger().info(f"Publishing shelf pose to MoveIt: {self.goal_shelf_pose_msg}\n")
        
        try:
            # 取得從 depth_camera 到 base_link 的轉換
            transform = self.tf_buffer.lookup_transform('base_link', 'depth_camera', rclpy.time.Time())
            # transform = self.tf_buffer.lookup_transform('base_link', 'camera_color_optical_frame', rclpy.time.Time())
            self.depth_camera_to_base_rot = transform.transform.rotation
            # self.get_logger().info(f'Transform: {transform}')
        except TransformException as e:
            self.get_logger().error(f'Could not get transform: {e}')
            return
        
        if self.k_received is not True:
            return
        self.goal_shelf_pose_msg = do_transform_pose(self.shelf_pose_msg.pose, transform)
        self.get_logger().info(f'架子座標：{self.goal_shelf_pose_msg}')
        
        """ xmin, ymin, xmax, ymax = self.bbox
        
        # 1. 取 ROI
        roi = depth_filtered[ymin:ymax, xmin:xmax].astype(np.float32)

        h, w = roi.shape

        # 2. 建立 pixel grid (ROI 座標)
        u = np.arange(w)
        v = np.arange(h)
        uu, vv = np.meshgrid(u, v)

        # 3. 深度轉 m
        z = roi

        # 4. 有效深度 mask (排除 0 以及距離大於 1.5 公尺的背景點)
        mask = (z > 0.01) & (z < 1.5)

        # 5. 轉成 3D point cloud
        # OpenCV 光學座標: x向右, y向下, z向前
        x_opt = (uu - self.cx + xmin) * z / self.fx
        y_opt = (vv - self.cy + ymin) * z / self.fy
        z_opt = z

        # 轉換為 ROS 標準座標 (depth_camera frame): X向前, Y向左, Z向上
        x = z_opt
        y = -x_opt
        z = -y_opt
        # 麽你的不需要有負號 實體的不用不知道為什麽 還在查詢
        # x = z_opt
        # y = x_opt
        # z = y_opt

        points = np.stack(
            (x[mask], y[mask], z[mask]),
            axis=-1
        )

        # 6. Open3D point cloud
        target = o3d.geometry.PointCloud()
        target.points = o3d.utility.Vector3dVector(points)
        
        # 1. 下採樣 (Voxel Filter)，讓點雲均勻化，加快 ICP 速度並減少權重失衡
        target = target.voxel_down_sample(voxel_size=0.001) # 5mm 
        # 2. 濾除孤立雜訊點 (Statistical Outlier Removal)
        target, ind = target.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
        
        target.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.02, max_nn=30))
        
        if len(points) > 0:
            # points 的形狀是 (N, 3)，取平均後會得到一組 [X_avg, Y_avg, Z_avg]
            centroid = np.mean(points, axis=0)
            x_center = centroid[0]
            y_center = centroid[1]
            z_center = centroid[2]
            # --- Debug Log ---
            # self.get_logger().info(f"目標點雲中心 (Centroid): x={x_center:.3f}, y={y_center:.3f}, z={z_center:.3f}, 總點數={len(points)}")
        else:
            # 如果沒有有效點，給予預設值 (全 0)
            x_center, y_center, z_center = 0.0, 0.0, 0.0
            self.get_logger().warning("深度圖 ROI 內沒有有效點！")
            return
        
        if self.depth_camera_to_base_rot is not None:
            # R_base_camera 是從 depth_camera 到 base_link 的旋轉
            # 因為物體在 base_link 是正的，我們需要它的反向旋轉 (相機看物體的相對旋轉)
            # Quaternion 反向即取 -x, -y, -z
            q_inv = [
                -self.depth_camera_to_base_rot.x,
                -self.depth_camera_to_base_rot.y,
                -self.depth_camera_to_base_rot.z,
                self.depth_camera_to_base_rot.w
            ]
            R_init = R.from_quat(q_inv).as_matrix()
        else:
            R_init = np.eye(3)

        init = np.eye(4)
        init[:3, :3] = R_init
        # 將 CAD 模型稍微往後推 (ROS X軸為深度方向)，確保掃描點位於模型前方，
        # 避免掃描點落入模型內部而錯誤匹配到後表面
        init[0, 3] = x_center
        init[1, 3] = y_center
        init[2, 3] = z_center
        
        # 反向 ICP (Scan 匹配到 CAD Model)
        # 解決 full-to-partial ICP 導致的 3cm 深度誤差：
        # 若將完整的 CAD 模型匹配到部分掃描點雲，會造成模型被往前拉扯約一半厚度的誤差。
        # 將 scan 匹配到 CAD，掃描點自然只會尋找 CAD 模型前表面的最近點。
        init_inv = np.linalg.inv(init)
        
        # Stage 1: Coarse alignment (Point-to-Point)
        icp_coarse = o3d.pipelines.registration.registration_icp(
            target,             # Source: partial scan
            self.source_pcd,    # Target: full CAD model (with perfect normals)
            0.2,               # 20cm threshold
            init_inv,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=30)
        )
        
        # Stage 2: Fine alignment (Point-to-Plane)
        icp_result = o3d.pipelines.registration.registration_icp(
            target,
            self.source_pcd,
            0.05,               # 5cm threshold
            icp_coarse.transformation,
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=30)
        )
        
        # 將轉換矩陣反轉回 CAD 到 Scan 的方向
        T_scan_to_cad = icp_result.transformation
        T = np.linalg.inv(T_scan_to_cad)
        # self.get_logger().info(f"ICP 變換矩陣的平移 (Origin): x={T[0,3]:.3f}, y={T[1,3]:.3f}, z={T[2,3]:.3f}")
        
        self.shelf_pose_msg.header = "camera_color_optical_frame"
        
        # 柱子尖端在模型局部空間的座標 (將 mm 轉換為公尺 m)
        tip_local = np.array([0.031485, 0.000, 0.105222, 1.0])
        
        # 將尖端座標透過 T 矩陣轉換到 depth_camera 座標系
        tip_camera = T @ tip_local
        
        self.shelf_pose_msg.pose.position.x = float(tip_camera[0])
        self.shelf_pose_msg.pose.position.y = float(tip_camera[1])
        self.shelf_pose_msg.pose.position.z = float(tip_camera[2])
        
        rot = R.from_matrix(T[:3, :3])
        q = rot.as_quat()   # (x, y, z, w)

        self.shelf_pose_msg.pose.orientation.x = float(q[0])
        self.shelf_pose_msg.pose.orientation.y = float(q[1])
        self.shelf_pose_msg.pose.orientation.z = float(q[2])
        self.shelf_pose_msg.pose.orientation.w = float(q[3])
        
        # self.get_logger().info(f"Shelf Pose in depth_camera frame: {self.shelf_pose_msg}\n")
        self.get_logger().info(f"Publishing shelf pose to MoveIt: {self.goal_shelf_pose_msg}\n")
        
        try:
            # 取得從 depth_camera 到 base_link 的轉換
            transform = self.tf_buffer.lookup_transform('base_link', 'depth_camera', rclpy.time.Time())
            # transform = self.tf_buffer.lookup_transform('base_link', 'camera_color_optical_frame', rclpy.time.Time())
            self.depth_camera_to_base_rot = transform.transform.rotation
            # self.get_logger().info(f'Transform: {transform}')
        except TransformException as e:
            self.get_logger().error(f'Could not get transform: {e}')
            return
        
        if self.k_received is not True:
            return
        self.goal_shelf_pose_msg = do_transform_pose(self.shelf_pose_msg.pose, transform)
        self.get_logger().info(f'架子座標：{self.goal_shelf_pose_msg}') """

def main():
    rclpy.init()
    node = ShelfDetectorDemo()
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
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image, CameraInfo
# from geometry_msgs.msg import PoseStamped, Pose
# from tf2_geometry_msgs import do_transform_pose
# from tf2_ros import Buffer, TransformListener, TransformException

# import cv2
# import numpy as np
# import open3d as o3d
# from cv_bridge import CvBridge
# from ultralytics import YOLO
# from scipy.spatial.transform import Rotation as R
# from vision_interfaces.srv import ShelfCoodinate

# class ShelfPoseDetector(Node):
#     def __init__(self):
#         super().__init__('shelf_pose_detector')
        
#         # 1. 訂閱相機資訊與影像
#         self.depth_sub = self.create_subscription(Image, '/camera/depth/image_raw', self.depth_callback, 10)
#         self.rgb_sub = self.create_subscription(Image, '/camera/color/image_raw', self.camera_callback, 10)
#         self.info_sub = self.create_subscription(CameraInfo, '/camera/depth/camera_info', self.info_callback, 10)

#         # 2. 初始化工具
#         self.bridge = CvBridge()
#         # self.model = YOLO('/home/alan/Moveit2/ar4_mk_5_ws/src/vision_yolo_depth/yolo/new_gz_wheel_shelf.pt')
#         self.model = YOLO('/home/alan/Moveit2/ar4_mk_5_ws/src/vision_yolo_depth/yolo/real_shelf_best.pt')
#         self.tf_buffer = Buffer()
#         self.tf_listener = TransformListener(self.tf_buffer, self)
        
#         # 3. 加載 CAD 模型 (用於 ICP 配準)
#         self.mesh = o3d.io.read_triangle_mesh("/home/alan/Moveit2/ar4_mk_5_ws/src/vision_yolo_depth/model/wheel-holder-no.stl")
#         self.mesh.scale(0.001, center=(0, 0, 0))
#         self.mesh.compute_vertex_normals()
#         self.source_pcd = self.mesh.sample_points_uniformly(number_of_points=15000)

#         # 變數緩存
#         self.depth_image = None
#         self.k_received = False
#         self.bbox = None
#         self.goal_shelf_pose_base = None # 儲存轉換到 base_link 後的座標
#         self.camera_intrinsics = {'fx': 0, 'fy': 0, 'cx': 0, 'cy': 0}
#         self.cx = 0.0
#         self.cy = 0.0
#         self.fx = 0.0
#         self.fy = 0.0
        
#         self.yolo_detect_shelf_results = None
#         self.depth_camera_to_base_rot = None
        
#         self.shelf_pose_msg = PoseStamped()
#         self.goal_shelf_pose_msg = PoseStamped()

#         # 4. 服務：提供架子座標
#         self.srv = self.create_service(ShelfCoodinate, 'get_shelf_pose', self.get_shelf_pose_srv)

#     def info_callback(self, msg):
#         if not self.k_received:
#             self.camera_intrinsics['fx'] = msg.k[0]
#             self.camera_intrinsics['fy'] = msg.k[4]
#             self.camera_intrinsics['cx'] = msg.k[2]
#             self.camera_intrinsics['cy'] = msg.k[5]
#             self.fx = self.camera_intrinsics['fx']
#             self.fy = self.camera_intrinsics['fy']
#             self.cx = self.camera_intrinsics['cx']
#             self.cy = self.camera_intrinsics['cy']
#             self.k_received = True

#     def camera_callback(self, msg):
#         """ YOLO 偵測目標物體 """
#         if not self.k_received:
#             return
#         camera_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
#         self.wheel_results = self.model(camera_image, stream=True, conf=0.60, verbose=False)
        
#         # 等待深度圖像
#         if self.depth_image is None:
#             self.get_logger().warning("等待深度圖像...")
#             # rclpy.spin_once(self, timeout_sec=0.05)
#             return

#         h, w = self.depth_image.shape[:2]
#         self.best_box = None
#         self.best_conf = -1.0

#         annotated = None

#         if self.wheel_results is None:
#             self.get_logger().info(f'沒抓到')
#             return

#         for r in self.wheel_results:
#             annotated = r.plot()
#             if r.boxes is None or len(r.boxes) == 0:
#                 continue
#             boxes_data = r.boxes.data.cpu().numpy()
#             self.yolo_detect_shelf_results = True
#             for box in boxes_data:
#                 x1, y1, x2, y2, conf, cls = box

#                 # y_max_idx = max(0, min(int(y2), h - 1))
#                 # x_max_idx = max(0, min(int(x2), w - 1))
#                 # x_min_idx = max(0, min(int(x1), w - 1))

#                 # xmax_z = self.depth_image[y_max_idx, x_max_idx]
#                 # xmin_z = self.depth_image[y_max_idx, x_min_idx]

#                 # real_width = (x2 * xmax_z - x1 * xmin_z) / self.fx
#                 # if real_width < 0.111:
#                 #     continue
                
#                 if conf > self.best_conf:
#                     self.best_conf = conf
#                     self.best_box = box

#         if self.best_box is None:
#             self.wheel_results = None
#             self.get_logger().info(f'best_box沒有')
#             cv2.imshow("camera", camera_image)
#             cv2.waitKey(1)
#             return

#         self.bbox = [int(self.best_box[0]), int(self.best_box[1]), int(self.best_box[2]), int(self.best_box[3])]
        
#         # for r in wheel_results:
#         #     annotated_frame = r.plot()
        
#         cv2.imshow("camera", annotated)
#         cv2.waitKey(1)

#     def depth_callback(self, msg):
#         """ 處理深度資訊並計算 3D 座標 (ICP) """
#         # if self.depth_image is None: 
#         self.depth_image = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
#         depth_filtered = cv2.bilateralFilter(self.depth_image.astype(np.float32), 7, 0, 25)
#         # 或使用 Temporal filter (多幀平均)
        
#         if self.bbox is None or self.yolo_detect_shelf_results is None or self.k_received is not True:
#             self.get_logger().info(f"尚未偵測到目標或相機參數未就緒，無法進行 ICP 配準\nself.bbox: {self.bbox}, self.yolo_detect_shelf_results: {self.yolo_detect_shelf_results}, self.k_received: {self.k_received}")
#             depth_display = cv2.normalize(
#                 self.depth_image,
#                 None,
#                 0,
#                 255,
#                 cv2.NORM_MINMAX
#             )

#             cv2.imshow("depth", depth_display.astype(np.uint8))
#             cv2.waitKey(1)
#             return
        
#         xmin, ymin, xmax, ymax = self.bbox
#         # self.get_logger().info(f"偵測到目標，進行 ICP 配準，bbox: {self.bbox}")
        
#         # 1. 取 ROI
#         roi = depth_filtered[ymin:ymax, xmin:xmax].astype(np.float32)

#         h, w = roi.shape

#         # 2. 建立 pixel grid (ROI 座標)
#         u = np.arange(w)
#         v = np.arange(h)
#         uu, vv = np.meshgrid(u, v)

#         # 3. 深度轉 m
#         z = roi

#         # 4. 有效深度 mask (排除 0 以及距離大於 1.5 公尺的背景點)
#         mask = (z > 0.01) & (z < 1.5)

#         # 5. 轉成 3D point cloud
#         # OpenCV 光學座標: x向右, y向下, z向前
#         x_opt = (uu - self.cx + xmin) * z / self.fx
#         y_opt = (vv - self.cy + ymin) * z / self.fy
#         z_opt = z

#         # 轉換為 ROS 標準座標 (depth_camera frame): X向前, Y向左, Z向上
#         x = z_opt
#         y = -x_opt
#         z = -y_opt
#         # 麽你的不需要有負號 實體的不用不知道為什麽 還在查詢
#         # x = z_opt
#         # y = x_opt
#         # z = y_opt
#         self.get_logger().info(f'{x.shape}, {y.shape}, {z.shape}')
#         points = np.stack(
#             (x[mask], y[mask], z[mask]),
#             axis=-1
#         )

#         # 6. Open3D point cloud
#         target = o3d.geometry.PointCloud()
#         target.points = o3d.utility.Vector3dVector(points)
        
#         if len(points) > 0:
#             # points 的形狀是 (N, 3)，取平均後會得到一組 [X_avg, Y_avg, Z_avg]
#             centroid = np.mean(points, axis=0)
#             x_center = centroid[0]
#             y_center = centroid[1]
#             z_center = centroid[2]
#             # --- Debug Log ---
#             self.get_logger().info(f"目標點雲中心 (Centroid): x={x_center:.3f}, y={y_center:.3f}, z={z_center:.3f}, 總點數={len(points)}")
#         else:
#             # 如果沒有有效點，給予預設值 (全 0)
#             x_center, y_center, z_center = 0.0, 0.0, 0.0
#             self.get_logger().warning("深度圖 ROI 內沒有有效點！")
#             return
        
#         if self.depth_camera_to_base_rot is not None:
#             # R_base_camera 是從 depth_camera 到 base_link 的旋轉
#             # 因為物體在 base_link 是正的，我們需要它的反向旋轉 (相機看物體的相對旋轉)
#             # Quaternion 反向即取 -x, -y, -z
#             q_inv = [
#                 -self.depth_camera_to_base_rot.x,
#                 -self.depth_camera_to_base_rot.y,
#                 -self.depth_camera_to_base_rot.z,
#                 self.depth_camera_to_base_rot.w
#             ]
#             R_init = R.from_quat(q_inv).as_matrix()
#         else:
#             R_init = np.eye(3)

#         init = np.eye(4)
#         init[:3, :3] = R_init
#         # 將 CAD 模型稍微往後推 (ROS X軸為深度方向)，確保掃描點位於模型前方，
#         # 避免掃描點落入模型內部而錯誤匹配到後表面
#         init[0, 3] = x_center 
#         init[1, 3] = y_center
#         init[2, 3] = z_center
        
#         # 反向 ICP (Scan 匹配到 CAD Model)
#         # 解決 full-to-partial ICP 導致的 3cm 深度誤差：
#         # 若將完整的 CAD 模型匹配到部分掃描點雲，會造成模型被往前拉扯約一半厚度的誤差。
#         # 將 scan 匹配到 CAD，掃描點自然只會尋找 CAD 模型前表面的最近點。
#         init_inv = np.linalg.inv(init)
        
#         # Stage 1: Coarse alignment (Point-to-Point)
#         icp_coarse = o3d.pipelines.registration.registration_icp(
#             target,             # Source: partial scan
#             self.source_pcd,    # Target: full CAD model (with perfect normals)
#             0.2,               # 20cm threshold
#             init_inv,
#             o3d.pipelines.registration.TransformationEstimationPointToPoint(),
#             o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=30)
#         )
        
#         # Stage 2: Fine alignment (Point-to-Plane)
#         icp_result = o3d.pipelines.registration.registration_icp(
#             target,
#             self.source_pcd,
#             0.05,               # 5cm threshold
#             icp_coarse.transformation,
#             o3d.pipelines.registration.TransformationEstimationPointToPlane(),
#             o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=30)
#         )
        
#         # 將轉換矩陣反轉回 CAD 到 Scan 的方向
#         T_scan_to_cad = icp_result.transformation
#         T = np.linalg.inv(T_scan_to_cad)
#         # self.get_logger().info(f"ICP 變換矩陣的平移 (Origin): x={T[0,3]:.3f}, y={T[1,3]:.3f}, z={T[2,3]:.3f}")
        
#         self.shelf_pose_msg.header = "camera_color_optical_frame"
        
#         # 柱子尖端在模型局部空間的座標 (將 mm 轉換為公尺 m)
#         tip_local = np.array([0.031485, 0.000, 0.105222, 1.0])
        
#         # 將尖端座標透過 T 矩陣轉換到 depth_camera 座標系
#         tip_camera = T @ tip_local
        
#         self.shelf_pose_msg.pose.position.x = float(tip_camera[0])
#         self.shelf_pose_msg.pose.position.y = float(tip_camera[1])
#         self.shelf_pose_msg.pose.position.z = float(tip_camera[2])
        
#         rot = R.from_matrix(T[:3, :3])
#         q = rot.as_quat()   # (x, y, z, w)

#         self.shelf_pose_msg.pose.orientation.x = float(q[0])
#         self.shelf_pose_msg.pose.orientation.y = float(q[1])
#         self.shelf_pose_msg.pose.orientation.z = float(q[2])
#         self.shelf_pose_msg.pose.orientation.w = float(q[3])
        
#         self.get_logger().info(f"Shelf Pose in depth_camera frame: {self.shelf_pose_msg}\n")
#         # self.get_logger().info(f"Publishing shelf pose to MoveIt: {self.goal_shelf_pose_msg}\n")
        
#         try:
#             # 取得從 depth_camera 到 base_link 的轉換
#             # transform = self.tf_buffer.lookup_transform('base_link', 'depth_camera', rclpy.time.Time())
#             transform = self.tf_buffer.lookup_transform('base_link', 'camera_color_optical_frame', rclpy.time.Time())
#             self.depth_camera_to_base_rot = transform.transform.rotation
#             # self.get_logger().info(f'Transform: {transform}')
#         except TransformException as e:
#             self.get_logger().error(f'Could not get transform: {e}')
#             return
        
#         if self.k_received is not True:
#             return
#         self.goal_shelf_pose_msg = do_transform_pose(self.shelf_pose_msg.pose, transform)
#         # self.get_logger().info(f'架子座標：{self.goal_shelf_pose_msg}')

#     def get_shelf_pose_srv(self, request, response):
#         """ Service 回應函數 """
#         if self.goal_shelf_pose_base is None:
#             response.status_message = "fail - no pose detected"
#             return response

#         # 設定固定抓取姿態 (根據原程式需求)
#         response.shelf_pose = self.goal_shelf_pose_base
#         response.shelf_pose.orientation.x = 0.704
#         response.shelf_pose.orientation.y = 0.704
#         response.shelf_pose.orientation.z = 0.062
#         response.shelf_pose.orientation.w = 0.062
        
#         response.status_message = "success"
#         return response

# def main():
#     rclpy.init()
#     node = ShelfPoseDetector()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()