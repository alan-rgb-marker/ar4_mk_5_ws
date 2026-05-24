# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import sys
from urllib import response
import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from ultralytics import YOLO
import numpy as np

from sensor_msgs.msg import Image, CameraInfo, JointState
from geometry_msgs.msg import PoseStamped 
from vision_interfaces.srv import Armcoodinate

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

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
        self.k = None
        self.k_received = False
        
        # 相機影像
        self.camera_subscript_ = self.create_subscription(Image, '/depth_camera/image', self.camera_callback, 5)
        self.camera_bridge = CvBridge()
        self.camera_image = None
        
        # yolo模型 wheel
        self.model = YOLO('/home/alan/Moveit2/ar4_mk_5_ws/src/vision_yolo_depth/yolo/wheel_best.pt')
        
        # tf2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_timer = self.create_timer(0.5, self.tf_callback)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
                
        # 讀取joint state夾爪
        self.gripper_joint_state_subscript_ = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 5)
        self.gripper_state = 0.01
        
        # 發布座標
        self.arm_cood_srv = self.create_service(Armcoodinate, 'Armcoodinate', self.arm_coordinate_callback)

    def depth_camera_callback(self, msg):
        depth_image = self.depth_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        
        # 取中心點距離
        h, w = depth_image.shape
    
        self.center_x = w // 2
        self.center_y = h // 2

        self.distance = depth_image[self.center_y, self.center_x]

        # self.get_logger().info(f'Distance: {self.distance}')

        # 顯示深度影像（需要正規化）
        depth_display = cv2.normalize(
            depth_image,
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
            self.k_received = True
        # self.get_logger().info(f'Depth camera info received: {self.k}')
        
    def camera_callback(self, msg):
        camera_image = self.camera_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        wheel_results = self.model(camera_image, stream=True, conf=0.80)
        
        for r in wheel_results:
            annotated_frame = r.plot()
        
        cv2.imshow("camera", annotated_frame)
        cv2.waitKey(1)
    
    def tf_callback(self):
        try:
            # 取得從 depth_camera 到 base_link 的轉換
            transform = self.tf_buffer.lookup_transform('base_link', 'depth_camera', rclpy.time.Time())
            # self.get_logger().info(f'Transform: {transform}')
        except TransformException as e:
            self.get_logger().error(f'Could not get transform: {e}')
            return

        if self.k_received:
            self.x = transform.transform.translation.x + abs(self.center_x - self.k[2]) * self.distance / self.k[0]  # 假設 k[0] 是焦距 fx k[2] 是主點 cx
            self.y = transform.transform.translation.y + abs(self.center_y - self.k[5]) * self.distance / self.k[4]  # 假設 k[4] 是焦距 fy k[5] 是主點 cy
            z = transform.transform.translation.z - self.distance
            self.get_logger().info(f"tf距離z： {z}")
            if z < 0.1562 and self.gripper_state <= 0.1:
                self.z = 0.1562
            elif z < 0.1515 and self.gripper_state > 0.1:
                self.z = 0.1515
            else:
                self.z = z + 0.08 # 增加8公分避免夾爪碰撞
            self.get_logger().info(f"tf距離self.z： {self.z}")
            w = transform.transform.rotation.w
            # self.get_logger().info(f"座標： x: {self.x}, y: {self.y}, z: {self.z}, w: {w}")
            # self.get_logger().info(f"距離： {self.distance}")
            # self.get_logger().info(f"轉換tf距離： {transform.transform.translation.z}")

    def joint_state_callback(self, msg):
        # 這裡可以解析夾爪的關節狀態，例如開合程度等
        index = msg.name.index('grip_to_base1')
        self.gripper_state = msg.position[index]  # 假設 position 包含夾爪的狀態
        # self.get_logger().info(f'Gripper joint state: {self.gripper_state}')

    def arm_coordinate_callback(self, request, response):
        if request.result == 'get_coordinates':
            pose = PoseStamped()
            pose.header.frame_id = "base_link"
            pose.pose.position.x = self.x
            pose.pose.position.y = self.y
            pose.pose.position.z = self.z
            # pose.pose.position.x = 0.3
            # pose.pose.position.y = 0.0
            # pose.pose.position.z = 0.1562


            pose.pose.orientation.x = 0.707
            pose.pose.orientation.y = 0.707
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 0.0
            self.get_logger().info(f"座標： x: {self.x}, y: {self.y}, z: {self.z}")
            response.arm_cood = pose   # ✔ 正確

            return response
        else:
            self.get_logger().error('Invalid request for arm coordinates')
            response.arm_cood = PoseStamped()
            # response.result = "error"
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
