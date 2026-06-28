import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, JointState
from geometry_msgs.msg import PoseStamped, Pose, PointStamped
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import Buffer, TransformListener, TransformException
import tf2_geometry_msgs
from std_srvs.srv import Trigger

import cv2
import numpy as np
import open3d as o3d
from cv_bridge import CvBridge
from ultralytics import YOLO
from scipy.spatial.transform import Rotation as R
from vision_interfaces.srv import Armcoodinate, ShelfCoodinate
from dataclasses import dataclass, field
from typing import Optional


# =========================================================
# State Dataclasses — 所有狀態集中在這裡，一目瞭然
# =========================================================

@dataclass
class CameraState:
    """相機內參與影像，兩個 detector 共用。"""
    k_received: bool = False
    cx: float = 0.0
    cy: float = 0.0
    fx: float = 0.0
    fy: float = 0.0
    depth_image: Optional[np.ndarray] = None


@dataclass
class ShelfState:
    """貨架偵測相關的所有狀態。"""
    # YOLO 結果
    yolo_results: object = None           # ultralytics Results (stream 最後一筆)

    # 位姿
    pose_msg: PoseStamped = field(default_factory=PoseStamped)
    goal_pose: Pose = field(default_factory=Pose)   # base_link 座標系

    # 觀測位置
    view_coord: Pose = field(default_factory=Pose)

    # 偵測控制
    start_detect_pose: bool = False
    delay_counter: int = 0              # 延遲啟動速度估算的 frame 計數

    # 速度估算
    vel: float = 0.0
    vel_is_detected: bool = False
    vel_measuring: bool = False
    vel_init_y: float = 0.0
    vel_start_time: float = 0.0

    def __post_init__(self):
        self.view_coord.position.x = 0.3
        self.view_coord.position.y = 0.0
        self.view_coord.position.z = 0.4
        self.view_coord.orientation.x = 0.704
        self.view_coord.orientation.y = 0.704
        self.view_coord.orientation.z = 0.062
        self.view_coord.orientation.w = 0.062


@dataclass
class WheelState:
    """輪胎偵測相關的所有狀態。"""
    bbox: Optional[list] = None          # [xmin, ymin, xmax, ymax]
    yolo_results: object = None

    pose_camera: Pose = field(default_factory=Pose)   # 相機座標系
    pose_base: Pose = field(default_factory=Pose)     # base_link 座標系
    distance: float = 0.0

    gripper_state: float = 0.01


# =========================================================
# ShelfDetector
# =========================================================

class ShelfDetector:
    """
    貨架偵測邏輯。
    所有狀態讀寫都透過 ShelfState，不直接持有 node 引用以外的狀態。
    """

    def __init__(self, model_path: str, node: Node,
                 cam: CameraState, state: ShelfState):
        self._node = node
        self._cam = cam
        self.state = state
        self._model = YOLO(model_path)

    # ----------------------------------------------------------
    # 主要偵測
    # ----------------------------------------------------------

    def detect(self, frame, tf_buffer) -> tuple:
        """
        對 frame 執行 YOLO-Pose 推理，更新 state.goal_pose。
        回傳 (results, annotated_frame | None)
        """
        results = self._model(frame, stream=True, conf=0.8, verbose=False)
        annotated_frame = None

        for r in results:
            annotated_frame = r.plot()

            if r.keypoints is None or len(r.keypoints.data) == 0:
                continue

            keypoints = r.keypoints.data[0].cpu().numpy()
            target_kp = keypoints[0]
            px, py, conf = target_kp

            if conf <= 0.5:
                continue

            u, v = int(px), int(py)
            cv2.circle(annotated_frame, (u, v), 1, (0, 0, 255), -1)

            if not self._pixel_in_bounds(u, v):
                continue

            x_depth = self._cam.depth_image[v, u]
            if x_depth <= 0.0:
                self._node.get_logger().warning('該點深度值為 0，可能反光或超出測距範圍。')
                continue

            camera_point = self._deproject(u, v, x_depth)
            self._transform_to_base(camera_point, tf_buffer)

        self.state.yolo_results = results
        return results, annotated_frame

    # ----------------------------------------------------------
    # 速度估算 (非阻塞狀態機)
    # ----------------------------------------------------------

    def detect_vel(self):
        """
        呼叫一次推進一步狀態機。
        狀態變化：vel_measuring False → True → (dt>=1s) vel_is_detected True
        """
        if not self.state.vel_measuring:
            self.state.vel_init_y = self.state.goal_pose.position.y
            self.state.vel_start_time = self._node.get_clock().now().nanoseconds / 1e9
            self.state.vel_measuring = True
            self._node.get_logger().info(
                f'開始測量貨架速度，初始 y={self.state.vel_init_y:.3f}'
            )
            return

        if self.state.vel_is_detected:
            return

        dt = self._node.get_clock().now().nanoseconds / 1e9 - self.state.vel_start_time
        if dt >= 1.0:
            dy = self.state.goal_pose.position.y - self.state.vel_init_y
            self.state.vel = dy / dt
            self.state.vel_is_detected = True
            self._node.get_logger().info(f'貨架速度：{self.state.vel:.4f} m/s')

    # ----------------------------------------------------------
    # 內部工具
    # ----------------------------------------------------------

    def _pixel_in_bounds(self, u: int, v: int) -> bool:
        h, w = self._cam.depth_image.shape
        return 0 <= v < h and 0 <= u < w

    def _deproject(self, u: int, v: int, x: float) -> Pose:
        """像素 + 深度 → 相機光學座標系 Pose。"""
        cam = self._cam
        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(-1.0 * (u - cam.cx) * x / cam.fx)
        pose.position.z = float(-1.0 * (v - cam.cy) * x / cam.fy)
        return pose

    def _transform_to_base(self, camera_point: Pose, tf_buffer):
        """轉換至 base_link，成功時更新 state.goal_pose。"""
        try:
            transform = tf_buffer.lookup_transform(
                'base_link',
                'camera_color_optical_frame',
                rclpy.time.Time()
            )
            base_point = do_transform_pose(camera_point, transform)
            self.state.goal_pose = base_point
            self._node.get_logger().info(
                f"抓取點在 base_link 的座標 -> "
                f"X: {base_point.position.x:.3f}, "
                f"Y: {base_point.position.y:.3f}, "
                f"Z: {base_point.position.z:.3f}"
            )
        except TransformException as ex:
            self._node.get_logger().warning(f"TF 轉換失敗: {ex}")


# =========================================================
# WheelDetector
# =========================================================

class WheelDetector:
    """
    輪胎偵測邏輯。
    所有狀態讀寫都透過 WheelState。
    """

    def __init__(self, model_path: str, node: Node,
                 cam: CameraState, state: WheelState):
        self._node = node
        self._cam = cam
        self.state = state
        self._model = YOLO(model_path)

    # ----------------------------------------------------------
    # 主要偵測
    # ----------------------------------------------------------

    def detect(self, frame) -> tuple:
        """
        YOLO 推理，更新 state.bbox。
        回傳 (bbox | None, annotated_frame | None)
        """
        stream = self._model(frame, stream=True, conf=0.60, verbose=False)
        bbox, annotated = self._pick_best_box(stream)
        self.state.bbox = bbox
        self.state.yolo_results = None  # stream 已消耗，重置
        return bbox, annotated

    def update_pose(self, tf_buffer):
        """
        根據最新 state.bbox 計算輪胎在 base_link 的座標，更新 state.pose_base。
        """
        cam = self._cam
        if cam.depth_image is None or self.state.bbox is None or not cam.k_received:
            return

        h, w = cam.depth_image.shape
        xmin, ymin, xmax, ymax = self.state.bbox
        raw_cx = int((xmin + xmax) / 2)
        raw_cy = int((ymin + ymax) / 2)
        cx = max(0, min(raw_cx, w - 1))
        cy = max(0, min(raw_cy, h - 1))

        dist = cam.depth_image[cy, cx]
        if dist <= 0.0:
            self._node.get_logger().warning('輪胎中心深度值無效，跳過。')
            return

        self.state.distance = dist
        self.state.pose_camera = self._deproject(cx, cy, dist)

        try:
            transform = tf_buffer.lookup_transform(
                'base_link', 'camera_color_optical_frame', rclpy.time.Time()
            )
            self.state.pose_base = do_transform_pose(self.state.pose_camera, transform)
        except TransformException as e:
            self._node.get_logger().error(f'TF lookup 輪胎失敗: {e}')

    # ----------------------------------------------------------
    # 內部工具
    # ----------------------------------------------------------

    def _pick_best_box(self, results_stream) -> tuple:
        """從 YOLO stream 選出信心值最高的 bbox。"""
        best_box = None
        best_conf = -1.0
        annotated = None

        for r in results_stream:
            annotated = r.plot()
            if r.boxes is None or len(r.boxes) == 0:
                continue
            for box in r.boxes.data.cpu().numpy():
                x1, y1, x2, y2, conf, cls = box
                if conf > best_conf:
                    best_conf = conf
                    best_box = box

        if best_box is None:
            return None, None

        bbox = [int(best_box[0]), int(best_box[1]),
                int(best_box[2]), int(best_box[3])]
        return bbox, annotated

    def _deproject(self, u: int, v: int, dist: float) -> Pose:
        """像素 + 深度 → 相機光學座標系 Pose。"""
        cam = self._cam
        pose = Pose()
        pose.position.x = dist
        pose.position.y = -1.0 * (u - cam.cx) * dist / cam.fx
        pose.position.z = -1.0 * (v - cam.cy) * dist / cam.fy
        pose.orientation.w = 1.0
        return pose


# =========================================================
# VisionDetectorNode — 只負責 ROS 串接
# =========================================================

class VisionDetectorNode(Node):

    def __init__(self):
        super().__init__('vision_detector')

        # =========================
        # 輪胎或架子開關
        # =========================
        self.detect_wheel = True
        # self.detect_wheel = False

        # =========================
        # 共用：相機狀態
        # =========================
        self._cam = CameraState()
        self._depth_bridge = CvBridge()
        self._camera_bridge = CvBridge()

        self.depth_info_sub = self.create_subscription(
            CameraInfo, '/camera/color/camera_info',
            self._camera_info_callback, 5
        )
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth/image_raw',
            self._depth_callback, 10
        )
        self.color_sub = self.create_subscription(
            Image, '/camera/color/image_raw',
            self._color_callback, 5
        )

        # =========================
        # TF2 (共用)
        # =========================
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # =========================
        # 貨架偵測器
        # =========================
        self._shelf_state = ShelfState()
        self._shelf_detector = ShelfDetector(
            model_path='/home/alan/Moveit2/ar4_mk_5_ws/src/vision_yolo_depth/yolo/real_shelf_pose_one_point_best.pt',
            node=self,
            cam=self._cam,
            state=self._shelf_state,
        )

        # =========================
        # 輪胎偵測器
        # =========================
        self._wheel_state = WheelState()
        self._wheel_detector = WheelDetector(
            model_path='/home/alan/Moveit2/ar4_mk_5_ws/src/vision_yolo_depth/yolo/real_wheel_best.pt',
            node=self,
            cam=self._cam,
            state=self._wheel_state,
        )

        # =========================
        # 夾爪 Joint State
        # =========================
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states',
            self._joint_state_callback, 5
        )

        # =========================
        # Services
        # =========================
        self.create_service(Armcoodinate,   'view_shelf_coord',  self._view_shelf_coord_callback)
        self.create_service(ShelfCoodinate, 'shelf_coord',       self._shelf_coord_callback)
        self.create_service(Armcoodinate,   'wheel_pose',        self._wheel_pose_callback)
        self.create_service(Trigger, 'reset_all_states', self._reset_callback)

    # =========================================================
    # 共用 Callbacks
    # =========================================================

    def _camera_info_callback(self, msg):
        if self._cam.k_received:
            return
        k = msg.k
        self._cam.cx = k[2]
        self._cam.cy = k[5]
        self._cam.fx = k[0]
        self._cam.fy = k[4]
        self._cam.k_received = True

    def _depth_callback(self, msg):
        depth = self._depth_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        if msg.encoding == '16UC1':
            depth = depth.astype(np.float32) / 1000.0
        else:
            depth = depth.astype(np.float32)
        self._cam.depth_image = depth

    def _color_callback(self, msg):
        if not self._cam.k_received or self._cam.depth_image is None:
            self.get_logger().warning('等待相機內參或深度影像...')
            return

        frame = self._camera_bridge.imgmsg_to_cv2(msg, 'bgr8')
        annotated = frame.copy()

        if not self.detect_wheel:
            # --- 貨架 ---
            _, annotated_frame = self._shelf_detector.detect(frame, self.tf_buffer)
            if annotated_frame is not None:
                annotated = annotated_frame

            if self._shelf_state.start_detect_pose:
                if self._shelf_state.delay_counter > 2:
                    self._shelf_detector.detect_vel()
                self._shelf_state.delay_counter += 1
        else:
            # --- 輪胎 ---
            _, wheel_annotated = self._wheel_detector.detect(frame)
            if wheel_annotated is not None:
                annotated = wheel_annotated
            self._wheel_detector.update_pose(self.tf_buffer)

        cv2.imshow('camera', annotated)
        cv2.waitKey(1)

    def _joint_state_callback(self, msg):
        if 'grip_to_base1' in msg.name:
            idx = msg.name.index('grip_to_base1')
            self._wheel_state.gripper_state = msg.position[idx]
        
            
        try:
            view_shelf_transform = self.tf_buffer.lookup_transform(
                'base_link',
                'gripper_tcp',
                rclpy.time.Time()
            )
        except TransformException as e:
            # 找不到就先跳出，等下一次 callback
            return False
        
        current_pose = Pose()
        current_pose.position.x = round(view_shelf_transform.transform.translation.x, 4)
        current_pose.position.y = round(view_shelf_transform.transform.translation.y, 4)
        current_pose.position.z = round(view_shelf_transform.transform.translation.z, 4)
        
        if abs(current_pose.position.x - self._shelf_state.view_coord.position.x) < 0.001 and \
           abs(current_pose.position.y - self._shelf_state.view_coord.position.y) < 0.001 and \
           abs(current_pose.position.z - self._shelf_state.view_coord.position.z) < 0.001:
            self._shelf_state.start_detect_pose = True

    # =========================================================
    # Services
    # =========================================================

    def _view_shelf_coord_callback(self, request, response):
        response.arm_cood = self._shelf_state.view_coord
        return response

    def _shelf_coord_callback(self, request, response):
        # 注意：在 spin thread 裡 busy-wait 會卡死，這裡保留原本行為
        # 建議未來改成：若尚未偵測到則直接回傳 status_message='not_ready'
        while self._shelf_state.yolo_results is None:
            self.get_logger().warning(
                f'尚未偵測到貨架，{self._shelf_state.yolo_results}'
            )

        while not isinstance(self._shelf_state.goal_pose, Pose):
            self.get_logger().warning('尚未取得貨架座標，等待中...')
        
        if request.req_cmd != 'get_shelf_cood':
            self.get_logger().warning(f'無效指令: {request.req_cmd}')
            return response
        
        goal = self._shelf_state.goal_pose
        goal.orientation.x = 0.704
        goal.orientation.y = 0.704
        goal.orientation.z = 0.062
        goal.orientation.w = 0.062

        response.start_pos_time = self.get_clock().now().to_msg()
        response.shelf_pose = goal
        response.status_message = 'success'
        response.shelf_vel = self._shelf_state.vel

        self.get_logger().info(f'回傳貨架座標: {goal}')
        return response

    def _wheel_pose_callback(self, request, response):
        if request.result != 'get_wheel_pose':
            self.get_logger().error('無效的輪胎座標請求')
            response.arm_cood = Pose()
            return response

        pose_base = self._wheel_state.pose_base
        goal = Pose()
        goal.orientation.x = 1.0
        goal.orientation.y = 0.0
        goal.orientation.z = 0.0
        goal.orientation.w = 0.0
        goal.position.x = pose_base.position.x # - 0.05
        goal.position.y = pose_base.position.y # - 0.03
        goal.position.z = pose_base.position.z + 0.02

        self.detect_wheel = False  # 停止輪胎偵測，避免干擾抓取

        response.arm_cood = goal
        self.get_logger().info(f'回傳輪胎座標: {goal}')
        return response
    
    def _reset_callback(self, request, response):
        """執行完一次任務後，重新初始化所有容易出問題的狀態"""
        # 貨架相關重置
        self._shelf_state.yolo_results = None
        self._shelf_state.vel_is_detected = False
        self._shelf_state.vel_measuring = False
        self._shelf_state.delay_counter = 0
        self._shelf_state.vel = 0.0
        self._shelf_state.start_detect_pose = False
        
        # 輪胎相關重置
        self._wheel_state.bbox = None
        self._wheel_state.pose_base = Pose()
        self._wheel_state.distance = 0.0
        
        # 模式切換（避免永久鎖死）
        self.detect_wheel = True
        
        self.get_logger().info("=== 所有狀態已重新初始化 ===")
        
        response.success = True
        response.message = 'All states reset successfully'
        return response


# =========================================================
# Main
# =========================================================

def main():
    rclpy.init()
    node = VisionDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image, CameraInfo, JointState
# from geometry_msgs.msg import PoseStamped, Pose, PointStamped
# from tf2_geometry_msgs import do_transform_pose
# from tf2_ros import Buffer, TransformListener, TransformException
# import tf2_geometry_msgs

# import cv2
# import numpy as np
# import open3d as o3d
# from cv_bridge import CvBridge
# from ultralytics import YOLO
# from scipy.spatial.transform import Rotation as R
# from vision_interfaces.srv import Armcoodinate, ShelfCoodinate


# class VisionDetectorNode(Node):

#     def __init__(self):
#         super().__init__('vision_detector')
#         # =========================
#         # 輪胎或架子開關
#         # =========================
#         self.detect_wheel = False #
        
        
#         # =========================
#         # 相機內參 (共用)
#         # =========================
#         self.k_received = False
#         self.cx = self.cy = self.fx = self.fy = 0.0

#         self.depth_info_sub = self.create_subscription(
#             CameraInfo,
#             '/camera/color/camera_info',
#             self._camera_info_callback,
#             5
#         )

#         # =========================
#         # 深度影像 (共用)
#         # =========================
#         self.depth_image = None
#         self.depth_bridge = CvBridge()

#         self.depth_sub = self.create_subscription(
#             Image,
#             '/camera/depth/image_raw',
#             self._depth_callback,
#             10
#         )

#         # =========================
#         # RGB 影像 (共用)
#         # =========================
#         self.camera_bridge = CvBridge()

#         self.color_sub = self.create_subscription(
#             Image,
#             '/camera/color/image_raw',
#             self._color_callback,
#             5
#         )

#         # =========================
#         # TF2 (共用)
#         # =========================
#         self.tf_buffer = Buffer()
#         self.tf_listener = TransformListener(self.tf_buffer, self)

#         # =========================
#         # YOLO — 貨架 (shelf)
#         # =========================
#         # self.shelf_model = YOLO(
#             # '/home/alan/Moveit2/ar4_mk_5_ws/src/vision_yolo_depth/yolo/real_shelf_best.pt'
#         # )
#         self.shelf_model = YOLO('/home/alan/Moveit2/ar4_mk_5_ws/src/vision_yolo_depth/yolo/real_shelf_pose_best.pt')
#         self.shelf_bbox = None
#         self.shelf_yolo_results = None

#         # =========================
#         # YOLO — 輪胎 (wheel)
#         # =========================
#         self.wheel_model = YOLO(
#             '/home/alan/Moveit2/ar4_mk_5_ws/src/vision_yolo_depth/yolo/real_wheel_best.pt'
#         )
#         self.wheel_bbox = None
#         self.wheel_results = None

#         # =========================
#         # 貨架偵測狀態
#         # =========================
#         self.shelf_pose_msg = PoseStamped()
#         self.goal_shelf_pose_msg = Pose()
#         self.depth_camera_to_base_rot = None
#         self.start_detect_shelf_pose = False
#         self.delay_start_detect_shelf_vel = 0

#         # 觀測位置 (view pose)
#         self.view_shelf_coord = Pose()
#         self.view_shelf_coord.position.x = 0.3
#         self.view_shelf_coord.position.y = 0.0
#         self.view_shelf_coord.position.z = 0.4
#         self.view_shelf_coord.orientation.x = 0.704
#         self.view_shelf_coord.orientation.y = 0.704
#         self.view_shelf_coord.orientation.z = 0.062
#         self.view_shelf_coord.orientation.w = 0.062

#         # 速度估算
#         self.shelf_vel = 0.0
#         self.shelf_vel_is_detected = False
#         self.vel_measuring = False
#         self.vel_init_y = 0.0
#         self.vel_start_time = 0.0

#         # =========================
#         # 輪胎偵測狀態
#         # =========================
#         self.wheel_pose_msg = Pose()   # 相機座標系
#         self.wheel_pose = Pose()       # base_link 座標系
#         self.wheel_center_x = 0.0
#         self.wheel_center_y = 0.0
#         self.wheel_distance = 0.0
#         self.gripper_state = 0.01

#         # =========================
#         # 夾爪 Joint State
#         # =========================
#         self.gripper_sub = self.create_subscription(
#             JointState,
#             '/joint_states',
#             self._joint_state_callback,
#             5
#         )

#         # =========================
#         # Services
#         # =========================
#         self.create_service(Armcoodinate,   'view_shelf_coord',  self._view_shelf_coord_callback)
#         self.create_service(ShelfCoodinate, 'shelf_coord',       self._shelf_coord_callback)
#         self.create_service(Armcoodinate,   'wheel_pose',        self._wheel_pose_callback)

#     # =========================================================
#     # 共用 Callbacks
#     # =========================================================

#     def _camera_info_callback(self, msg):
#         if self.k_received:
#             return
#         k = msg.k
#         self.cx = k[2]
#         self.cy = k[5]
#         self.fx = k[0]
#         self.fy = k[4]
#         self.k_received = True

#     def _depth_callback(self, msg):
#         depth = self.depth_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
#         if msg.encoding == '16UC1':
#             depth = depth.astype(np.float32) / 1000.0
#         else:
#             depth = depth.astype(np.float32)
#         self.depth_image = depth

#     def _color_callback(self, msg):
#         if not self.k_received or self.depth_image is None:
#             self.get_logger().warning('等待相機內參或深度影像...')
#             return

#         frame = self.camera_bridge.imgmsg_to_cv2(msg, 'bgr8')
#         annotated = frame.copy()

#         if self.detect_wheel is False:
#             # --- 貨架 YOLO ---
#             self.shelf_yolo_results, annotated_frame = self.detect_shelf_pose(frame, msg)
#             if self.start_detect_shelf_pose:
#                 if self.delay_start_detect_shelf_vel > 2:
#                     self._detect_shelf_vel()
#                 self.delay_start_detect_shelf_vel += 1
#             if annotated_frame is not None:
#                 annotated = annotated_frame
#         else:
#             # --- 輪胎 YOLO ---
#             wheel_stream = self.wheel_model(frame, stream=True, conf=0.60, verbose=False)
#             self.wheel_bbox, wheel_annotated = self._pick_best_box(wheel_stream, frame)
#             if wheel_annotated is not None:
#                 annotated = wheel_annotated
#             # --- 輪胎流程 ---
#             self._update_wheel_pose()

#         cv2.imshow('camera', annotated)
#         cv2.waitKey(1)

#     def _joint_state_callback(self, msg):
#         if 'grip_to_base1' in msg.name:
#             idx = msg.name.index('grip_to_base1')
#             self.gripper_state = msg.position[idx]

#     # =========================================================
#     # 共用工具
#     # =========================================================

#     def _pick_best_box(self, results_stream, fallback_frame):
#         """
#         從 YOLO stream 中選出信心值最高的 bbox。
#         回傳 (bbox_list | None, annotated_frame | None)
#         """
#         best_box = None
#         best_conf = -1.0
#         annotated = None

#         for r in results_stream:
#             annotated = r.plot()
#             if r.boxes is None or len(r.boxes) == 0:
#                 continue
#             for box in r.boxes.data.cpu().numpy():
#                 x1, y1, x2, y2, conf, cls = box
#                 if conf > best_conf:
#                     best_conf = conf
#                     best_box = box

#         if best_box is None:
#             return None, None

#         bbox = [int(best_box[0]), int(best_box[1]), int(best_box[2]), int(best_box[3])]
#         return bbox, annotated

#     def _lookup_transform(self, target, source):
#         """查詢 TF，失敗回傳 None。"""
#         try:
#             return self.tf_buffer.lookup_transform(target, source, rclpy.time.Time())
#         except TransformException as e:
#             self.get_logger().error(f'TF lookup {source}→{target} 失敗: {e}')
#             return None

#     # =========================================================
#     # 貨架偵測
#     # =========================================================

#     def _update_shelf_view_status(self):
#         """確認手臂是否到達觀測位置。"""
#         if self.shelf_yolo_results is None:
#             return
#         transform = self._lookup_transform('base_link', 'gripper_tcp')
#         if transform is None:
#             return
#         t = transform.transform.translation
#         dx = abs(t.x - self.view_shelf_coord.position.x)
#         dy = abs(t.y - self.view_shelf_coord.position.y)
#         dz = abs(t.z - self.view_shelf_coord.position.z)
#         self.start_detect_shelf_pose = (dx < 0.001 and dy < 0.001 and dz < 0.001)

#     def detect_shelf_pose(self, frame, msg):
#         # 執行 YOLO-Pose 推理
#         results = self.shelf_model(frame, stream=True, conf=0.8, verbose=False)

#         for r in results:
#             annotated_frame = r.plot()  # 畫出預設的骨架與框
            
#             # 確保有偵測到關鍵點 (Pose)
#             if r.keypoints is not None and len(r.keypoints.data) > 0:
                
#                 # 這裡假設抓取畫面上第一個物件的「第 0 個」關鍵點作為 Demo
#                 # 資料結構通常是: [物件數量, 關鍵點數量, 3 (x, y, conf)]
#                 keypoints = r.keypoints.data[0].cpu().numpy()
#                 target_kp = keypoints[0]  # 取得你想要的特定點 (依你的模型定義更改 index)
                
#                 px, py, conf = target_kp

#                 # 信心度大於閾值才採算
#                 if conf > 0.5:
#                     u, v = int(px), int(py)
#                     cv2.circle(annotated_frame, (u, v), 1, (0, 0, 255), -1) # 畫紅點標示

#                     # --- 步驟 A: 從 2D 像素推算 3D 相機座標 ---
#                     # 避免超出影像邊界
#                     if 0 <= v < self.depth_image.shape[0] and 0 <= u < self.depth_image.shape[1]:
#                         # z = self.depth_image[v, u]
#                         x = self.depth_image[v, u]
                        
#                         if x > 0.0: # 深度不能為 0

#                             # 根據相機針孔模型公式 (Deprojection)
#                             y = -1 * (u - self.cx) * x / self.fx
#                             z = -1 * (v - self.cy) * x / self.fy

#                             # --- 步驟 B: 建立 PointStamped 準備 TF 轉換 ---
#                             camera_point = Pose()
#                             camera_point.position.x = float(x)
#                             camera_point.position.y = float(y)
#                             camera_point.position.z = float(z)

#                             # --- 步驟 C: 轉換至 base_link 座標系 ---
#                             try:
#                                 # 查詢 base_link 到 相機 frame 的變換矩陣
#                                 transform = self.tf_buffer.lookup_transform(
#                                     'base_link',
#                                     'camera_color_optical_frame',
#                                     rclpy.time.Time()
#                                 )
                                
#                                 # 執行轉換
#                                 base_link_point = do_transform_pose(camera_point, transform)
#                                 self.goal_shelf_pose_msg = base_link_point  # 更新全域目標座標
                                
#                                 # 顯示最終要的 base_link 座標
#                                 self.get_logger().info(
#                                     f"抓取點在 base_link 的座標 -> "
#                                     f"X: {base_link_point.position.x:.3f}, "
#                                     f"Y: {base_link_point.position.y:.3f}, "
#                                     f"Z: {base_link_point.position.z:.3f}"
#                                 )

#                             except TransformException as ex:
#                                 self.get_logger().warning(f"TF 轉換失敗: {ex}")
#                                 return None, frame
#                         else:
#                             self.get_logger().warning("該點深度值為 0，可能反光或超出測距範圍。")
#                             return None, frame
#         return results, annotated_frame

#     def _detect_shelf_vel(self):
#         """非阻塞式速度估算 (狀態機)。"""
#         if not self.vel_measuring:
#             self.vel_init_y = self.goal_shelf_pose_msg.position.y
#             self.vel_start_time = self.get_clock().now().nanoseconds / 1e9
#             self.vel_measuring = True
#             self.get_logger().info(f'開始測量貨架速度，初始 y={self.vel_init_y:.3f}')
#         else:
#             dt = self.get_clock().now().nanoseconds / 1e9 - self.vel_start_time
#             if dt >= 1.0 and not self.shelf_vel_is_detected:
#                 dy = self.goal_shelf_pose_msg.position.y - self.vel_init_y
#                 self.shelf_vel = dy / dt
#                 self.get_logger().info(f'貨架速度：{self.shelf_vel:.4f} m/s')
#                 self.shelf_vel_is_detected = True

#     # =========================================================
#     # 輪胎偵測
#     # =========================================================

#     def _update_wheel_pose(self):
#         """根據最新 wheel_bbox 計算輪胎在 base_link 的座標。"""
#         if self.depth_image is None or self.wheel_bbox is None or not self.k_received:
#             return

#         h, w = self.depth_image.shape
#         xmin, ymin, xmax, ymax = self.wheel_bbox
#         raw_cx = int((xmin + xmax) / 2)
#         raw_cy = int((ymin + ymax) / 2)
#         cx = max(0, min(raw_cx, w - 1))
#         cy = max(0, min(raw_cy, h - 1))

#         dist = self.depth_image[cy, cx]
#         if dist <= 0.0:
#             self.get_logger().warning('輪胎中心深度值無效，跳過。')
#             return
#         self.wheel_distance = dist

#         # 光學座標系 → ROS 座標系 (X前, Y左, Z上)
#         self.wheel_pose_msg.position.x = dist
#         self.wheel_pose_msg.position.y = -1.0 * (cx - self.cx) * dist / self.fx
#         self.wheel_pose_msg.position.z = -1.0 * (cy - self.cy) * dist / self.fy
#         self.wheel_pose_msg.orientation.w = 1.0

#         transform = self._lookup_transform('base_link', 'camera_color_optical_frame')
#         if transform is None:
#             return
#         self.wheel_pose = do_transform_pose(self.wheel_pose_msg, transform)

#     # =========================================================
#     # Services
#     # =========================================================

#     def _view_shelf_coord_callback(self, request, response):
#         self.start_detect_shelf_pose = True
#         response.arm_cood = self.view_shelf_coord
#         return response

#     def _shelf_coord_callback(self, request, response):
#         while self.shelf_yolo_results is None:
#             self.get_logger().warning(f'尚未偵測到貨架，{self.shelf_yolo_results}')

#         while not isinstance(self.goal_shelf_pose_msg, Pose):
#             self.get_logger().warning('尚未取得貨架座標，等待中...')

#         if request.req_cmd != 'get_shelf_cood':
#             self.get_logger().warning(f'無效指令: {request.req_cmd}')
#             return response

#         # self._detect_shelf_pose()

#         goal = self.goal_shelf_pose_msg
#         # 固定抓取姿態
#         goal.orientation.x = 0.704
#         goal.orientation.y = 0.704
#         goal.orientation.z = 0.062
#         goal.orientation.w = 0.062

#         response.start_pos_time = self.get_clock().now().to_msg()
#         response.shelf_pose = goal
#         response.status_message = 'success'
#         response.shelf_vel = self.shelf_vel

#         self.get_logger().info(f'回傳貨架座標: {goal}')
#         return response

#     def _wheel_pose_callback(self, request, response):
#         if request.result != 'get_wheel_pose':
#             self.get_logger().error('無效的輪胎座標請求')
#             response.arm_cood = Pose()
#             return response

#         goal = Pose()
#         goal.orientation.x = 1.0
#         goal.orientation.y = 0.0
#         goal.orientation.z = 0.0
#         goal.orientation.w = 0.0
#         goal.position.x = self.wheel_pose.position.x - 0.05
#         goal.position.y = self.wheel_pose.position.y - 0.03
#         goal.position.z = self.wheel_pose.position.z + 0.025
        
#         self.detect_wheel = False  # 停止輪胎偵測，避免干擾抓取

#         response.arm_cood = goal
#         self.get_logger().info(f'回傳輪胎座標: {goal}')
#         return response


# # =========================================================
# # Main
# # =========================================================

# def main():
#     rclpy.init()
#     node = VisionDetectorNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()
#     cv2.destroyAllWindows()


# if __name__ == '__main__':
#     main()