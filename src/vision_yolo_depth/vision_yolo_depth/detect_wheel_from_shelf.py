"""
wheel_detector_node.py
-----------------------
功能：
  1. 訂閱 RGB / Depth / CameraInfo
  2. 用 YOLO 偵測輪胎 bbox 中心點深度，反投影至 3D
  3. 透過 TF 轉換到 base_link 座標系
  4. 兩幀差分計算 base_link Y 軸速度
  5. 提供兩個 ROS 2 service：
       - view_shelf_coord  → 回傳觀測位置（讓手臂移過去），同時開啟偵測
       - shelf_coord       → 回傳最新輪胎位置 + Y 軸速度

Topics（不改名）：
  SUB  /camera/color/image_raw
  SUB  /camera/depth/image_raw
  SUB  /camera/color/camera_info

Services：
  SRV  view_shelf_coord  →  vision_interfaces/srv/Armcoodinate
  SRV  shelf_coord       →  vision_interfaces/srv/ShelfCoodinate
"""

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import Buffer, TransformListener, TransformException
from std_srvs.srv import Trigger

from vision_interfaces.srv import Armcoodinate, ShelfCoodinate

from cv_bridge import CvBridge
from ultralytics import YOLO

import cv2
import numpy as np
from dataclasses import dataclass, field
from typing import Optional
from collections import deque


# =========================================================
# 可調參數
# =========================================================

YOLO_MODEL_PATH    = "/home/alan/Moveit2/ar4_mk_5_ws/src/vision_yolo_depth/yolo/real_wheel_on_shelf_best.pt"
YOLO_CONF          = 0.5
DEPTH_MEDIAN_KSIZE = 5       # bbox 中心區塊 median kernel（奇數）
DEPTH_VALID_MIN    = 0.05    # 最小有效深度 (m)
DEPTH_VALID_MAX    = 3.0     # 最大有效深度 (m)

# 手臂移到這裡之後，相機才能看到輪胎（base_link 座標）
VIEW_POSE = dict(
    x=0.28, y=0.000, z=0.350,
    qx=0.704, qy=0.704, qz=0.062, qw=0.062,
)


# =========================================================
# State Dataclasses
# =========================================================

@dataclass
class CameraState:
    """相機內參與最新影像。"""
    k_received:  bool            = False
    fx: float                    = 0.0
    fy: float                    = 0.0
    cx: float                    = 0.0
    cy: float                    = 0.0
    depth_image: Optional[np.ndarray] = None


@dataclass
class WheelState:
    """輪胎偵測的所有狀態。"""
    # 偵測開關（收到 view_shelf_coord 後才開）
    detecting: bool = False

    # 最新偵測結果（base_link 座標系）
    pose_base:    Optional[Pose]  = None
    stamp:        object          = None   # builtin_interfaces/Time

    # 速度估算（兩幀差分，base_link Y 軸）
    # deque 存 (timestamp_sec: float, y_base: float)
    vel_history:  deque           = field(default_factory=lambda: deque(maxlen=2))
    vel_y:        float           = 0.0

    # 觀測位置（固定值，由 view_shelf_coord 回傳）
    view_coord: Pose = field(default_factory=Pose)

    def __post_init__(self):
        v = self.view_coord
        v.position.x    = VIEW_POSE["x"]
        v.position.y    = VIEW_POSE["y"]
        v.position.z    = VIEW_POSE["z"]
        v.orientation.x = VIEW_POSE["qx"]
        v.orientation.y = VIEW_POSE["qy"]
        v.orientation.z = VIEW_POSE["qz"]
        v.orientation.w = VIEW_POSE["qw"]


# =========================================================
# WheelDetector — 純邏輯，不持有 Node 引用以外的 ROS 資源
# =========================================================

class WheelDetector:
    """
    YOLO 偵測 → 深度反投影 → TF 轉換 → 速度估算。
    所有狀態集中在 WheelState。
    """

    def __init__(self, model_path: str, node: Node,
                 cam: CameraState, state: WheelState):
        self._node  = node
        self._cam   = cam
        self.state  = state
        self._model = YOLO(model_path)
        self._half  = DEPTH_MEDIAN_KSIZE // 2   # depth patch 半寬

    # ----------------------------------------------------------
    # 主流程（每幀 RGB callback 呼叫）
    # ----------------------------------------------------------

    def process_frame(self, frame: np.ndarray,
                      stamp, tf_buffer: Buffer) -> Optional[np.ndarray]:
        """
        執行偵測、反投影、TF 轉換、速度更新。
        回傳帶標注的影像（用於 imshow），偵測失敗時回傳 None。
        """
        bbox, annotated = self._detect(frame)
        if bbox is None:
            return annotated

        u, v = self._bbox_center(bbox)
        depth_m = self._sample_depth(u, v)
        if depth_m is None:
            self._node.get_logger().warn(f"bbox 中心 ({u},{v}) 深度無效，跳過此幀")
            return annotated

        pose_camera = self._deproject(u, v, depth_m)
        pose_base   = self._transform_to_base(pose_camera, tf_buffer)
        if pose_base is None:
            return annotated

        # 更新速度（用 base_link Y 軸做兩幀差分）
        stamp_sec = stamp.sec + stamp.nanosec * 1e-9
        self._update_velocity(stamp_sec, pose_base.position.y)

        # 儲存結果
        self.state.pose_base = pose_base
        self.state.stamp     = stamp
        self.state.vel_y     = self._compute_velocity()

        self._node.get_logger().debug(
            f"輪胎 base_link: ({pose_base.position.x:.3f}, "
            f"{pose_base.position.y:.3f}, {pose_base.position.z:.3f})  "
            f"vel_y={self.state.vel_y:.4f} m/s"
        )
        return annotated

    # ----------------------------------------------------------
    # 內部工具
    # ----------------------------------------------------------

    def _detect(self, frame: np.ndarray) -> tuple:
        """YOLO 推理，挑信心最高的 bbox。回傳 (bbox_xyxy | None, annotated)。"""
        results_stream = self._model(frame, stream=True, conf=YOLO_CONF, verbose=False)

        best_box  = None
        best_conf = -1.0
        annotated = None

        for r in results_stream:
            annotated = r.plot()
            if r.boxes is None or len(r.boxes) == 0:
                continue
            for box in r.boxes.data.cpu().numpy():
                x1, y1, x2, y2, conf, _ = box
                if conf > best_conf:
                    best_conf = conf
                    best_box  = (int(x1), int(y1), int(x2), int(y2))

        return best_box, annotated

    @staticmethod
    def _bbox_center(bbox: tuple) -> tuple:
        x1, y1, x2, y2 = bbox
        return int((x1 + x2) / 2), int((y1 + y2) / 2)

    def _sample_depth(self, u: int, v: int) -> Optional[float]:
        """取 bbox 中心附近 patch 的中位數深度（公尺），無效回傳 None。"""
        depth = self._cam.depth_image
        if depth is None:
            return None

        h, w = depth.shape
        u0 = max(0, u - self._half);  u1 = min(w, u + self._half + 1)
        v0 = max(0, v - self._half);  v1 = min(h, v + self._half + 1)

        patch = depth[v0:v1, u0:u1]
        valid = patch[(patch > DEPTH_VALID_MIN) & (patch < DEPTH_VALID_MAX)]
        if valid.size == 0:
            return None
        return float(np.median(valid))

    def _deproject(self, u: int, v: int, depth_m: float) -> Pose:
        """
        像素 (u, v) + 深度 → camera_color_optical_frame 的 Pose。
        光學座標系慣例：X右、Y下、Z前
        → 轉成 ROS 慣例 X前、Y左、Z上 via (-y, -z, x) —
          但這裡直接塞進 Pose.position 讓 TF 處理即可，
          只要 frame_id 正確（camera_color_optical_frame），tf 會轉換。
        """
        cam = self._cam
        pose = Pose()
        # 光學座標系下的 3D 點
        pose.position.x = depth_m
        pose.position.y = -1.0 * (u - cam.cx) * depth_m / cam.fx
        pose.position.z = -1.0 * (v - cam.cy) * depth_m / cam.fy
        pose.orientation.w = 1.0
        return pose

    def _transform_to_base(self, camera_pose: Pose,
                            tf_buffer: Buffer) -> Optional[Pose]:
        """轉換至 base_link，失敗回傳 None。"""
        try:
            transform = tf_buffer.lookup_transform(
                'base_link',
                'camera_color_optical_frame',
                rclpy.time.Time()
            )
            return do_transform_pose(camera_pose, transform)
        except TransformException as ex:
            self._node.get_logger().warn(f"TF 轉換失敗: {ex}")
            return None

    """ def _update_velocity(self, stamp_sec: float, y_base: float):
        self.state.vel_history.append((stamp_sec, y_base)) """
    
    # def _compute_velocity(self) -> float:
    #     """兩幀差分，歷史不足時回傳 0.0。"""
    #     hist = self.state.vel_history
    #     if len(hist) < 2:
    #         return 0.0
    #     t0, y0 = hist[0]
    #     t1, y1 = hist[-1]
    #     dt = t1 - t0
    #     if dt < 1e-6:
    #         return 0.0
    #     print((y1 - y0) / dt)
    #     return (y1 - y0) / dt
    
    # VEL_SAMPLE_INTERVAL = 1.0   # 秒，放在檔案頂部常數區

    def _update_velocity(self, stamp_sec: float, y_base: float):
        """
        1 秒兩點差分：
        - vel_history 空的 → 記第一個錨點
        - 距第一個錨點 >= VEL_SAMPLE_INTERVAL → 記第二點，觸發速度計算，然後
          把第二點變成新的第一點（滑動窗口），下一秒繼續更新
        - 兩點之間的幀全部丟棄
        """
        hist = self.state.vel_history

        if len(hist) == 0:
            hist.append((stamp_sec, y_base))
            return

        t0, _ = hist[0]
        if stamp_sec - t0 < 1.0:
            return  # 還沒到 1 秒，忽略

        # 到達 1 秒：記第二點（maxlen=2 自動擠掉第一點）
        hist.append((stamp_sec, y_base))

    def _compute_velocity(self) -> float:
        hist = self.state.vel_history
        if len(hist) < 2:
            return 0.0
        t0, y0 = hist[0]
        t1, y1 = hist[1]
        dt = t1 - t0
        if dt < 1e-6:
            return 0.0
        return (y1 - y0) / dt


# =========================================================
# WheelDetectorNode — 只負責 ROS 串接
# =========================================================

class WheelDetectorNode(Node):

    def __init__(self):
        super().__init__("wheel_detector_node")

        # ── 共用：相機狀態 ────────────────────────────────────
        self._cam    = CameraState()
        self._bridge = CvBridge()

        # ── TF2 ──────────────────────────────────────────────
        self._tf_buffer   = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # ── 輪胎偵測器 ────────────────────────────────────────
        self._wheel_state    = WheelState()
        self._wheel_detector = WheelDetector(
            model_path=YOLO_MODEL_PATH,
            node=self,
            cam=self._cam,
            state=self._wheel_state,
        )

        # ── Subscribers ───────────────────────────────────────
        self.create_subscription(
            CameraInfo, "/camera/color/camera_info",
            self._cb_camera_info, 10)

        self.create_subscription(
            Image, "/camera/depth/image_raw",
            self._cb_depth, 10)

        self.create_subscription(
            Image, "/camera/color/image_raw",
            self._cb_color, 5)

        # ── Services ──────────────────────────────────────────
        self.create_service(
            Armcoodinate, "view_wheel_on_shelf_coord",
            self._srv_view_shelf_coord)

        self.create_service(
            ShelfCoodinate, "wheel_coord_on_shelf",
            self._srv_shelf_coord)
        
        self.create_service(
            Trigger, "reset_wheel_from_shelf", 
            self.reset_callback)

        self.get_logger().info("WheelDetectorNode 已啟動，等待影像與服務呼叫...")
        
        # -- 其他變數 --------------------------------------------
        self.if_start = False

    # =========================================================
    # Subscriber callbacks
    # =========================================================

    def _cb_camera_info(self, msg: CameraInfo):
        if self._cam.k_received:
            return
        k = msg.k
        self._cam.fx, self._cam.fy = k[0], k[4]
        self._cam.cx, self._cam.cy = k[2], k[5]
        self._cam.k_received = True
        self.get_logger().info(
            f"相機內參載入：fx={self._cam.fx:.1f}  fy={self._cam.fy:.1f}  "
            f"cx={self._cam.cx:.1f}  cy={self._cam.cy:.1f}"
        )

    def _cb_depth(self, msg: Image):
        raw = self._bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        if msg.encoding == "16UC1":
            self._cam.depth_image = raw.astype(np.float32) / 1000.0
        else:
            self._cam.depth_image = raw.astype(np.float32)

    def _cb_color(self, msg: Image):
        if not self._cam.k_received or self._cam.depth_image is None:
            self.get_logger().warning("等待相機內參或深度影像...")
            return

        # 偵測開關：未收到 view_shelf_coord 前不處理
        if not self._wheel_state.detecting:
            return

        frame = self._bridge.imgmsg_to_cv2(msg, "bgr8")
        
        if self.if_start is False:
            # cv2.imshow("Wheel Detection", annotated if annotated is not None else frame)
            # cv2.waitKey(1)
            return
        

        annotated = self._wheel_detector.process_frame(
            frame, msg.header.stamp, self._tf_buffer
        )

        # cv2.imshow("Wheel Detection", annotated if annotated is not None else frame)
        # cv2.waitKey(1)

    # =========================================================
    # Service handlers
    # =========================================================

    def _srv_view_shelf_coord(
        self,
        request:  Armcoodinate.Request,
        response: Armcoodinate.Response,
    ) -> Armcoodinate.Response:
        """
        Step 1：C++ 呼叫此 service，取得觀測位置並讓手臂移過去。
        同時清空舊偵測資料，打開偵測開關。
        """
        self.get_logger().info(f"收到 view_shelf_coord 請求：cmd={request.result}")

        # 重置狀態
        state = self._wheel_state
        state.pose_base  = None
        state.stamp      = None
        state.vel_y      = 0.0
        state.vel_history.clear()
        state.detecting  = True

        self.get_logger().info("偵測開關已開啟，開始記錄輪胎位置")

        response.arm_cood = state.view_coord
        self.get_logger().info(
            f"回傳觀測位置 x={state.view_coord.position.x}  "
            f"y={state.view_coord.position.y}  "
            f"z={state.view_coord.position.z}"
        )
        return response

    def _srv_shelf_coord(
        self,
        request:  ShelfCoodinate.Request,
        response: ShelfCoodinate.Response,
    ) -> ShelfCoodinate.Response:
        """
        Step 2：C++ 呼叫此 service，取得最新輪胎位置與 Y 軸速度。
        回傳後關閉偵測開關，避免繼續更新速度歷史。
        """
        self.get_logger().info(f"收到 wheel_coord_on_shelf 請求：cmd={request.req_cmd}")

        state = self._wheel_state

        if state.pose_base is None or state.stamp is None:
            self.get_logger().warn("尚未取得輪胎位置，回傳 no_detection")
            response.status_message = "no_detection"
            response.shelf_vel      = 0.0
            return response

        response.shelf_pose     = state.pose_base
        response.shelf_vel      = state.vel_y
        response.start_pos_time = state.stamp
        response.status_message = "ok"

        # 資料已交出，關閉偵測
        state.detecting = False
        self.if_start = False

        self.get_logger().info(
            f"回傳輪胎位置 "
            f"x={state.pose_base.position.x:.3f}  "
            f"y={state.pose_base.position.y:.3f}  "
            f"z={state.pose_base.position.z:.3f}  "
            f"vel_y={state.vel_y:.4f} m/s  → 偵測開關已關閉"
        )
        return response
    
    def reset_callback(self,
        request:  Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        state = self._wheel_state
        state.detecting   = False
        state.pose_base   = None
        state.stamp       = None
        state.vel_y       = 0.0
        state.vel_history.clear()
        
        self.if_start = True

        self.get_logger().info("=== WheelState 已重新初始化 ===")
        response.success = True
        response.message = "reset ok"
        return response


# =========================================================
# Entry point
# =========================================================

def main():
    rclpy.init()
    node = WheelDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()