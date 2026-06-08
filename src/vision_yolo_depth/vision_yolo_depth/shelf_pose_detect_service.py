import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, Pose
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import Buffer, TransformListener, TransformException
from std_srvs.srv import Trigger


import cv2
import numpy as np
import open3d as o3d
from cv_bridge import CvBridge
from ultralytics import YOLO
from scipy.spatial.transform import Rotation as R
from vision_interfaces.srv import Armcoodinate, ShelfCoodinate

class ShelfPoseDetector(Node):

    def __init__(self):
        super().__init__('gripper_to_shelf')
        
        # =========================
        # Depth image
        # =========================
        self.depth_subscript_ = self.create_subscription(
            Image,
            '/depth_camera/depth_image',
            self.depth_camera_callback,
            10
        )
        self.delay_start_detect_shelf_vel = 0

        self.depth_bridge = CvBridge()
        self.depth_image = None
        
        self.depth_info_subscript_ = self.create_subscription(CameraInfo, '/depth_camera/camera_info', self.depth_info_timer_callback, 5)
        self.k = None
        self.k_received = False
        self.cx = 0.0
        self.cy = 0.0
        self.fx = 0.0
        self.fy = 0.0
        self.shelf_pose_msg = PoseStamped()
        
        # =========================
        # RGB image
        # =========================
        self.camera_subscript_ = self.create_subscription(
            Image,
            '/depth_camera/image',
            self.camera_callback,
            5
        )
        
        self.camera_bridge = CvBridge()

        # =========================
        # YOLO
        # =========================
        self.model = YOLO(
            '/home/alan/Moveit2/ar4_mk_5_ws/src/vision_yolo_depth/yolo/new_gz_wheel_shelf.pt'
        )
        self.bbox = None
        self.yolo_detect_shelf_results = None
        
        # =========================
        # cad model
        # =========================
        # 相機內參設定 (Open3D 格式)
        self.mesh = o3d.io.read_triangle_mesh("/home/alan/Moveit2/ar4_mk_5_ws/src/vision_yolo_depth/model/wheel-holder-no.stl")
        self.mesh.scale(0.001, center=(0, 0, 0))
        # 計算法向量
        self.mesh.compute_vertex_normals()
        self.source_pcd = self.mesh.sample_points_uniformly(number_of_points=15000)
        # o3d.visualization.draw_geometries([self.source_pcd])
        self.intrinsics = None
        
        # ========================
        # tf2
        # ========================
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # self.tf_timer = self.create_timer(0.5, self.tf_callback)
        self.depth_camera_to_base_rot = None
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        
        # ========================
        # view shelf pose service
        # ========================
        self.view_shelf_coord_service_ = self.create_service(Armcoodinate, 'view_shelf_coord', self.view_shelf_coordinate_callback)
        self.view_shelf_coord = Pose()
        self.view_shelf_coord.position.x = 0.367
        self.view_shelf_coord.position.y = 0.000
        self.view_shelf_coord.position.z = 0.287
        self.view_shelf_coord.orientation.x = 0.704
        self.view_shelf_coord.orientation.y = 0.704
        self.view_shelf_coord.orientation.z = 0.062
        self.view_shelf_coord.orientation.w = 0.062

        # ========================
        # view shelf座標service
        # ========================
        self.shelf_pose_service_ = self.create_service(ShelfCoodinate, 'shelf_coord', self.shelf_pose_callback)
        self.goal_shelf_pose_msg = Pose()
        self.shelf_vel = 0.0
        self.shelf_vel_is_detected = False

        # ========================
        # 啟動手臂node client
        # ========================
        # self.start_arm_client_ = self.create_client(Trigger, 'run_service')
        
        # ========================
        # 其他變數
        # ========================
        self.if_detected_shelf_vel = False
        self.vel_measuring = False
        self.vel_init_y = 0.0
        self.vel_start_time = 0.0
        
        self.start_detect_shelf_pose = False
        
    # =========================
    # Depth callback
    # =========================
    def depth_camera_callback(self, msg): 
        self.depth_image = self.depth_bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='passthrough'
        )
        
        self.detect_view_pose()       
        if self.yolo_detect_shelf_results is None or not self.k_received or self.start_detect_shelf_pose is not True:
            """ depth_display = cv2.normalize(
                self.depth_image,
                None,
                0,
                255,
                cv2.NORM_MINMAX
            )
            cv2.imshow("depth", depth_display.astype(np.uint8))
            cv2.waitKey(1) """
            return

        self.detect_shelf_pose()
        if self.delay_start_detect_shelf_vel > 2:
            self.detect_shelf_vel()
        self.delay_start_detect_shelf_vel += 1
        # display
        """ depth_display = cv2.normalize(
            self.depth_image,
            None,
            0,
            255,
            cv2.NORM_MINMAX
        )

        cv2.imshow("depth", depth_display.astype(np.uint8))
        cv2.waitKey(1) """
        
    def depth_info_timer_callback(self, msg):
        # 這裡可以解析深度相機的內部參數，例如焦距、主點等
        # 這些參數對於將像素座標轉換為實際世界座標非常重要
        self.k = msg.k
        if not self.k_received or self.k is None:
            self.cx = self.k[2]
            self.cy = self.k[5]
            self.fx = self.k[0]
            self.fy = self.k[4]
            self.k_received = True
            # self.get_logger().info(f'Depth camera info received: {self.k}')
    
    # =========================
    # YOLO callback
    # =========================
    def camera_callback(self, msg):
        
        if self.depth_image is None or not self.k_received:
            self.get_logger().warning("等待深度圖像或相機內參，跳過此幀...")
            return

        frame = self.camera_bridge.imgmsg_to_cv2(msg, 'bgr8')

        # yolo_detect_shelf_results = self.model(frame, verbose=False)
        self.yolo_detect_shelf_results = self.model.predict(frame, conf=0.80, verbose=False)
                
        # ✔ FIX 4: 正確判斷 detection
        if len(self.yolo_detect_shelf_results[0].boxes) == 0:
            self.yolo_detect_shelf_results = None
            # cv2.imshow("camera", frame)
            # cv2.waitKey(1)
            return

        for r in self.yolo_detect_shelf_results:
            if len(r.boxes) == 0:
                continue
            boxes_data = r.boxes.data.cpu().numpy()

            # 運用 NumPy 的切片（Slicing）一次拿完所有欄位
            xyxys = boxes_data[0, :4]     # 所有的座標 (N, 4)
            confs = boxes_data[0, 4]      # 所有的信心度 (N,)
            clss  = boxes_data[0, 5].astype(int)  # 所有的類別 ID (N,)
            
            while self.depth_image is None:
                self.get_logger().warning("等待深度圖像...")
                rclpy.spin_once(self, timeout_sec=0.05)
            
            h, w = self.depth_image.shape[:2]
            y_max_idx = max(0, min(int(boxes_data[0][3]), h - 1))
            x_max_idx = max(0, min(int(boxes_data[0][2]), w - 1))
            x_min_idx = max(0, min(int(boxes_data[0][0]), w - 1))
            
            xmax_z = self.depth_image[y_max_idx, x_max_idx] 
            xmin_z = self.depth_image[y_max_idx, x_min_idx]
            real_max_width = boxes_data[0][2] * xmax_z / self.fx
            real_min_width = boxes_data[0][0] * xmin_z / self.fx
            real_width = real_max_width - real_min_width
            
            if real_width < 0.111:  # 如果實際寬度小於 11.1 公分，可能是誤檢，跳過
                # self.get_logger().warning(f"檢測到的物體寬度過小 (real_width={real_width:.3f} m)，可能是誤檢，已跳過")
                continue

            self.bbox = [int(boxes_data[0][0]), int(boxes_data[0][1]), int(boxes_data[0][2]), int(boxes_data[0][3])]
            # self.get_logger().info(f'xmin: {self.bbox[0]}, ymin: {self.bbox[1]}, xmax: {self.bbox[2]}, ymax: {self.bbox[3]}')
                
        annotated = self.yolo_detect_shelf_results[0].plot()
        # cv2.imshow("camera", annotated)
        # cv2.waitKey(1)
    
    def detect_view_pose(self):
        if self.yolo_detect_shelf_results is None:
            return False
        
        view_shelf_transform = self.tf_buffer.lookup_transform(
            'base_link',
            'gripper_tcp',
            rclpy.time.Time()
        )
        
        current_pose = Pose()
        current_pose.position.x = round(view_shelf_transform.transform.translation.x, 4)
        current_pose.position.y = round(view_shelf_transform.transform.translation.y, 4)
        current_pose.position.z = round(view_shelf_transform.transform.translation.z, 4)
        
        if abs(current_pose.position.x - self.view_shelf_coord.position.x) < 0.001 and \
           abs(current_pose.position.y - self.view_shelf_coord.position.y) < 0.001 and \
           abs(current_pose.position.z - self.view_shelf_coord.position.z) < 0.001:
            # self.get_logger().info("已達到觀看架子的位置，可以開始偵測架子座標了！")
            self.start_detect_shelf_pose = True
            return True
        else:
            # self.get_logger().info("尚未達到觀看架子的位置。")
            self.start_detect_shelf_pose = False
            return False
  
    def detect_shelf_vel(self):
        # 使用狀態機的方式，非阻塞地計算速度
        if not self.vel_measuring:
            # 第一次進入，記錄初始時間與位置
            self.vel_init_y = self.goal_shelf_pose_msg.position.y
            self.vel_start_time = self.get_clock().now().nanoseconds / 1e9
            self.vel_measuring = True
            # self.get_logger().info(f'架子座標： {self.goal_shelf_pose_msg}')
            self.get_logger().info(f"開始測量貨架速度，初始位置: {self.vel_init_y:.3f}")
        else:
            now = self.get_clock().now().nanoseconds / 1e9
            dt = now - self.vel_start_time

            if dt >= 1.0 and self.shelf_vel_is_detected is not True:
                current_y = self.goal_shelf_pose_msg.position.y
                d_current_y = current_y - self.vel_init_y
                
                self.shelf_vel = d_current_y / dt
                self.get_logger().info(f'架子速度：{self.shelf_vel} m/s')
                self.shelf_vel_is_detected = True
    
    def detect_shelf_pose(self):
        # 在取 ROI 前先做
        depth_filtered = cv2.bilateralFilter(self.depth_image.astype(np.float32), 7, 0, 25)
        # 或使用 Temporal filter (多幀平均)
        
        if self.bbox is None or self.yolo_detect_shelf_results is None or self.k_received is not True:
            depth_display = cv2.normalize(
                self.depth_image,
                None,
                0,
                255,
                cv2.NORM_MINMAX
            )

            cv2.imshow("depth", depth_display.astype(np.uint8))
            cv2.waitKey(1)
            return
        
        xmin, ymin, xmax, ymax = self.bbox
        
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

        points = np.stack(
            (x[mask], y[mask], z[mask]),
            axis=-1
        )

        # 6. Open3D point cloud
        target = o3d.geometry.PointCloud()
        target.points = o3d.utility.Vector3dVector(points)
        
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
        init[0, 3] = x_center + 0.04
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
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=50)
        )
        
        # Stage 2: Fine alignment (Point-to-Plane)
        icp_result = o3d.pipelines.registration.registration_icp(
            target,
            self.source_pcd,
            0.05,               # 5cm threshold
            icp_coarse.transformation,
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=50)
        )
        
        # 將轉換矩陣反轉回 CAD 到 Scan 的方向
        T_scan_to_cad = icp_result.transformation
        T = np.linalg.inv(T_scan_to_cad)
        # self.get_logger().info(f"ICP 變換矩陣的平移 (Origin): x={T[0,3]:.3f}, y={T[1,3]:.3f}, z={T[2,3]:.3f}")
        
        self.shelf_pose_msg.header = "depth_camera"
        
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
            self.depth_camera_to_base_rot = transform.transform.rotation
            # self.get_logger().info(f'Transform: {transform}')
        except TransformException as e:
            self.get_logger().error(f'Could not get transform: {e}')
            return
        
        if self.k_received is not True:
            return
        self.goal_shelf_pose_msg = do_transform_pose(self.shelf_pose_msg.pose, transform)

    def view_shelf_coordinate_callback(self, request, response):
        request.result = "get_view_shelf_coord"
        
        response.arm_cood = self.view_shelf_coord
        
        # 可以偵測架子的位置
        self.start_detect_shelf_pose = True
        
        # self.get_logger().info(f"Received view_shelf_coord request, responding with shelf pose: {self.view_shelf_coord}\n")
        return response
    
    def shelf_pose_callback(self, request, response):
        
        while self.yolo_detect_shelf_results is None:
            pass
        # Ensure we have a transformed PoseStamped available
        while not isinstance(self.goal_shelf_pose_msg, Pose):
            self.get_logger().warning("No transformed shelf pose available to publish.")
            pass
        while request.req_cmd != "get_shelf_cood":
            self.get_logger().warning("Invalid request command.")
            pass

        self.detect_shelf_pose()
        start_time = self.get_clock().now().to_msg()
        
        self.goal_shelf_pose_msg.orientation.x = 0.704
        self.goal_shelf_pose_msg.orientation.y = 0.704
        self.goal_shelf_pose_msg.orientation.z = 0.062
        self.goal_shelf_pose_msg.orientation.w = 0.062
                    
        response.start_pos_time = start_time
        response.shelf_pose = self.goal_shelf_pose_msg
        response.status_message = "success"
        # response.shelf_vel = 0.0
        response.shelf_vel = self.shelf_vel

        # self.get_logger().info(f"Publishing shelf pose to MoveIt: {self.goal_shelf_pose_msg}\n")
        return response
        

        
    
def main():
    rclpy.init()

    node = ShelfPoseDetector()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()