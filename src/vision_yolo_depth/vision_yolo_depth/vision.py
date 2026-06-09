import sys
from urllib import response
import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from ultralytics import YOLO
import numpy as np

from sensor_msgs.msg import Image, CameraInfo, JointState
from geometry_msgs.msg import PoseStamped, Pose
from vision_interfaces.srv import Armcoodinate

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_geometry_msgs import do_transform_pose
from scipy.spatial.transform import Rotation as R

class DepthSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        
        # 深度影像影像
        self.depth_subscript_ = self.create_subscription(Image, '/depth_camera/depth_image', self.depth_camera_callback, 5)
        self.depth_bridge = CvBridge()
        self.depth_image = None
        self.center_x = 0.0
        self.center_y = 0.0
        self.distance = 0.0
        
        # 深度相機資訊 計算像素和實際座標的轉換
        self.depth_info_subscript_ = self.create_subscription(CameraInfo, '/depth_camera/camera_info', self.depth_info_timer_callback, 5)
        self.k_received = False
        self.k = None
        self.cx = 0.0
        self.cy = 0.0
        self.fx = 0.0
        self.fy = 0.0
        
        # 相機影像
        self.camera_subscript_ = self.create_subscription(Image, '/depth_camera/image', self.camera_callback, 5)
        self.camera_bridge = CvBridge()
        self.camera_image = None
        
        # yolo模型 wheel
        # self.model = YOLO('/home/alan/Moveit2/ar4_mk_5_ws/src/vision_yolo_depth/yolo/wheel_best.pt')
        self.model = YOLO(
            '/home/alan/Moveit2/ar4_mk_5_ws/src/vision_yolo_depth/yolo/new_gz_wheel_shelf.pt'
        )
        self.wheel_results = None
        self.best_box = None
        self.best_conf = -1.0
        self.bbox = None
        
        # tf2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.wheel_pose = Pose()     # base_link到輪胎的座標
        self.wheel_pose_msg = PoseStamped() # 深度相機到輪胎的座標
                
        # 讀取joint state夾爪
        self.gripper_joint_state_subscript_ = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 5)
        self.gripper_state = 0.01
        
        # 發布座標
        self.arm_cood_srv = self.create_service(Armcoodinate, 'wheel_pose', self.arm_coordinate_callback)

    def depth_camera_callback(self, msg):
        self.depth_image = self.depth_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        
        # 取中心點距離
        h, w = self.depth_image.shape
        
        if self.bbox is not None and self.depth_image is not None:
            
            xmin, ymin, xmax, ymax = self.bbox

            self.center_x = int((xmin + xmax) / 2)
            self.center_y = int((ymin + ymax) / 2)

            self.distance = self.depth_image[self.center_y, self.center_x]
        
        try:
            # 取得從 depth_camera 到 base_link 的轉換
            transform = self.tf_buffer.lookup_transform('base_link', 'depth_camera', rclpy.time.Time())
            # self.get_logger().info(f'Transform: {transform}')
        except TransformException as e:
            self.get_logger().error(f'Could not get transform: {e}')
            return

        if self.k_received is not True:
            return
        transform = self.tf_buffer.lookup_transform('base_link', 'depth_camera', rclpy.time.Time())
        
        self.wheel_pose_msg.pose.position.y = -1 * (self.center_x - self.cx) * self.distance / self.fx  # 假設 k[0] 是焦距 fx k[2] 是主點 cx
        
        self.wheel_pose_msg.pose.position.z = -1 * (self.center_y - self.cy) * self.distance / self.fy  # 假設 k[4] 是焦距 fy k[5] 是主點 cy
        self.wheel_pose_msg.pose.position.x = self.distance
        
        self.wheel_pose = do_transform_pose(self.wheel_pose_msg.pose,transform)
        
        # self.get_logger().info(f"輪胎座標: {self.wheel_pose}")

        # self.get_logger().info(f'Distance: {self.distance}')

        # 顯示深度影像（需要正規化）
        depth_display = cv2.normalize(
            self.depth_image,
            None,
            0,
            255,
            cv2.NORM_MINMAX
        )

        depth_display = np.uint8(depth_display)

        cv2.imshow("depth", depth_display)
        cv2.waitKey(1)
    
    def depth_info_timer_callback(self, msg):
        # 這裡可以解析深度相機的內部參數，例如焦距、主點等
        # 這些參數對於將像素座標轉換為實際世界座標非常重要
        self.k = msg.k
        if not self.k_received and self.k is not None:
            self.cx = self.k[2]
            self.cy = self.k[5]
            self.fx = self.k[0]
            self.fy = self.k[4]
            self.k_received = True
        # self.get_logger().info(f'Depth camera info received: {self.k}')
        
    def camera_callback(self, msg):
        if not self.k_received:
            return
        camera_image = self.camera_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        self.wheel_results = self.model(camera_image, stream=True, conf=0.80, verbose=False)
        
        # 等待深度圖像
        while self.depth_image is None:
            self.get_logger().warning("等待深度圖像...")
            # rclpy.spin_once(self, timeout_sec=0.05)

        h, w = self.depth_image.shape[:2]
        self.best_box = None
        self.best_conf = -1.0

        annotated = None

        if self.wheel_results is None:
            self.get_logger().info(f'沒抓到')
            return

        for r in self.wheel_results:
            annotated = r.plot()
            if r.boxes is None or len(r.boxes) == 0:
                continue
            boxes_data = r.boxes.data.cpu().numpy()

            for box in boxes_data:
                x1, y1, x2, y2, conf, cls = box

                y_max_idx = max(0, min(int(y2), h - 1))
                x_max_idx = max(0, min(int(x2), w - 1))
                x_min_idx = max(0, min(int(x1), w - 1))

                xmax_z = self.depth_image[y_max_idx, x_max_idx]
                xmin_z = self.depth_image[y_max_idx, x_min_idx]

                real_width = (x2 * xmax_z - x1 * xmin_z) / self.fx
                if real_width < 0.111:
                    continue
                
                if conf > self.best_conf:
                    self.best_conf = conf
                    self.best_box = box

        if self.best_box is None:
            self.wheel_results = None
            return

        self.bbox = [int(self.best_box[0]), int(self.best_box[1]), int(self.best_box[2]), int(self.best_box[3])]
        
        # for r in wheel_results:
        #     annotated_frame = r.plot()
        
        cv2.imshow("camera", annotated)
        cv2.waitKey(1)

    def joint_state_callback(self, msg):
        # 這裡可以解析夾爪的關節狀態，例如開合程度等
        index = msg.name.index('grip_to_base1')
        self.gripper_state = msg.position[index]  # 假設 position 包含夾爪的狀態
        # self.get_logger().info(f'Gripper joint state: {self.gripper_state}')

    def arm_coordinate_callback(self, request, response):
        if request.result == 'get_wheel_pose':
            # self.get_logger().info(f"座標： x: {self.x}, y: {self.y}, z: {self.z}")
            
            goal_wheel_pose = Pose()
        
            # 回傳實際偵測到的輪胎 pose，不做額外 y 偏移，也保留原始 orientation

            goal_wheel_pose.orientation.x = 1.0
            goal_wheel_pose.orientation.y = 0.0
            goal_wheel_pose.orientation.z = 0.0
            goal_wheel_pose.orientation.w = 0.0
            
            goal_wheel_pose.position.x = self.wheel_pose.position.x
            goal_wheel_pose.position.y = self.wheel_pose.position.y

            goal_wheel_pose.position.z = self.wheel_pose.position.z
            self.get_logger().info(f'實際： {self.wheel_pose}')

            self.get_logger().info(f'輪胎座標；{goal_wheel_pose}')
            response.arm_cood = goal_wheel_pose

            return response
        else:
            self.get_logger().error('Invalid request for arm coordinates')
            response.arm_cood = Pose()
            return response
    
def main(args=None):
    rclpy.init(args=args)

    node = DepthSubscriber()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
