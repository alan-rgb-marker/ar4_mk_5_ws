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
        
        # 1. 訂閱相機資訊與影像
        self.depth_sub = self.create_subscription(Image, '/depth_camera/depth_image', self.depth_callback, 10)
        self.rgb_sub = self.create_subscription(Image, '/depth_camera/image', self.camera_callback, 10)
        self.info_sub = self.create_subscription(CameraInfo, '/depth_camera/camera_info', self.info_callback, 10)

        # 2. 初始化工具
        self.bridge = CvBridge()
        self.model = YOLO('/home/alan/Moveit2/ar4_mk_5_ws/src/vision_yolo_depth/yolo/new_gz_wheel_shelf.pt')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 3. 加載 CAD 模型 (用於 ICP 配準)
        self.mesh = o3d.io.read_triangle_mesh("/home/alan/Moveit2/ar4_mk_5_ws/src/vision_yolo_depth/model/wheel-holder-no.stl")
        self.mesh.scale(0.001, center=(0, 0, 0))
        self.mesh.compute_vertex_normals()
        self.source_pcd = self.mesh.sample_points_uniformly(number_of_points=15000)

        # 變數緩存
        self.depth_image = None
        self.k_received = False
        self.bbox = None
        self.goal_shelf_pose_base = None # 儲存轉換到 base_link 後的座標
        self.camera_intrinsics = {'fx': 0, 'fy': 0, 'cx': 0, 'cy': 0}

        # 4. 服務：提供架子座標
        self.srv = self.create_service(ShelfCoodinate, 'get_shelf_pose', self.get_shelf_pose_srv)

    def info_callback(self, msg):
        if not self.k_received:
            self.camera_intrinsics['fx'] = msg.k[0]
            self.camera_intrinsics['fy'] = msg.k[4]
            self.camera_intrinsics['cx'] = msg.k[2]
            self.camera_intrinsics['cy'] = msg.k[5]
            self.k_received = True

    def camera_callback(self, msg):
        """ YOLO 偵測目標物體 """
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        results = self.model.predict(frame, conf=0.80, verbose=False)
        
        if len(results[0].boxes) > 0:
            box = results[0].boxes.data.cpu().numpy()[0]
            self.bbox = [int(box[0]), int(box[1]), int(box[2]), int(box[3])]
        else:
            self.bbox = None

    def depth_callback(self, msg):
        """ 處理深度資訊並計算 3D 座標 (ICP) """
        if self.depth_image is None: self.depth_image = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
        if self.bbox is None or not self.k_received: return

        # 取出 ROI 並轉為點雲
        xmin, ymin, xmax, ymax = self.bbox
        roi_z = self.depth_image[ymin:ymax, xmin:xmax].astype(np.float32)
        mask = (roi_z > 0.01) & (roi_z < 1.5)

        u, v = np.meshgrid(np.arange(roi_z.shape[1]), np.arange(roi_z.shape[0]))
        x_opt = (u - self.camera_intrinsics['cx'] + xmin) * roi_z / self.camera_intrinsics['fx']
        y_opt = (v - self.camera_intrinsics['cy'] + ymin) * roi_z / self.camera_intrinsics['fy']
        
        # 轉換為 ROS 座標系 (X向前)
        points = np.stack((roi_z[mask], -x_opt[mask], -y_opt[mask]), axis=-1)
        if len(points) < 10: return

        target_pcd = o3d.geometry.PointCloud()
        target_pcd.points = o3d.utility.Vector3dVector(points)
        centroid = np.mean(points, axis=0)

        # ICP 配準 (簡化流程)
        init_trans = np.eye(4)
        init_trans[:3, 3] = [centroid[0] + 0.04, centroid[1], centroid[2]]
        
        reg_result = o3d.pipelines.registration.registration_icp(
            target_pcd, self.source_pcd, 0.05, np.linalg.inv(init_trans),
            o3d.pipelines.registration.TransformationEstimationPointToPlane()
        )
        
        T = np.linalg.inv(reg_result.transformation)
        
        # 計算抓取點 (Tip Offset)
        tip_local = np.array([0.031485, 0.000, 0.105222, 1.0])
        tip_camera = T @ tip_local

        # 封裝 PoseStamped
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = 'depth_camera'
        pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z = tip_camera[:3]
        
        q = R.from_matrix(T[:3, :3]).as_quat()
        pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z, pose_msg.pose.orientation.w = q

        # TF 轉換到 base_link
        try:
            transform = self.tf_buffer.lookup_transform('base_link', 'depth_camera', rclpy.time.Time())
            self.goal_shelf_pose_base = do_transform_pose(pose_msg.pose, transform)
            self.get_logger().info(f"成功更新 base_link 坐標: shelf pose={self.goal_shelf_pose_base}")
        except TransformException as e:
            self.get_logger().error(f"TF 轉換失敗: {e}")

    def get_shelf_pose_srv(self, request, response):
        """ Service 回應函數 """
        if self.goal_shelf_pose_base is None:
            response.status_message = "fail - no pose detected"
            return response

        # 設定固定抓取姿態 (根據原程式需求)
        response.shelf_pose = self.goal_shelf_pose_base
        response.shelf_pose.orientation.x = 0.704
        response.shelf_pose.orientation.y = 0.704
        response.shelf_pose.orientation.z = 0.062
        response.shelf_pose.orientation.w = 0.062
        
        response.status_message = "success"
        return response

def main():
    rclpy.init()
    node = ShelfPoseDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()