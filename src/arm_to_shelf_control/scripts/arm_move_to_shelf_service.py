#!/usr/bin/env python3

import sys
from urllib import response
import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from ultralytics import YOLO
import numpy as np

from sensor_msgs.msg import Image, CameraInfo, JointState
from geometry_msgs.msg import PoseStamped, TwistStamped, Pose 
from vision_interfaces.srv import Armcoodinate

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_geometry_msgs import do_transform_pose
from scipy.spatial.transform import Rotation as R
from moveit_msgs.msg import ServoStatus
from simple_pid import PID

class ArmMoveToShelfClient(Node):

    def __init__(self):
        super().__init__('arm_move_to_shelf_client')
        
        self.goal_arm_cood_subscriber_ = self.create_subscription(Pose, 'shelf_Armcoodinate', self.arm_coordinate_callback, 10)
        self.goal_shelf_pose_msg = Pose()
        self.get_coord = True
        
        # ========================
        # tf2
        # ========================
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_timer = self.create_timer(0.5, self.tf_callback)
        self.current_shelf_pose_msg = PoseStamped()
        
        # ==========================
        # publisher控制移動
        # ==========================
        self.moveit_servo_publisher_ = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        timer_period = 0.1  # 提高頻率到 10Hz 以滿足 MoveIt Servo 持續輸入的需求
        self.moveit_servo_timer = self.create_timer(timer_period, self.moveit_servo_callback)
        self.kp_pos = 1.5
        self.kp_rot = 2.0

        self.max_linear_vel = 0.15      # m/s
        self.max_angular_vel = 0.8      # rad/s

        self.position_deadband = 0.005  # 5mm
        self.rotation_deadband = 0.02   # rad
        
        # ==========================
        # moveit2 servo狀態
        # ==========================
        self.servo_status_subscriber_ = self.create_subscription(ServoStatus, '/servo_node/status', self.servo_status_callback, 10)
        self.current_servo_status = ServoStatus()
        
        
    def arm_coordinate_callback(self, msg):
        if self.get_coord is True:
            self.goal_shelf_pose_msg = msg
            self.get_logger().info(f"Received goal shelf pose: {self.goal_shelf_pose_msg}\n")
            self.get_coord = False
        # else:
        #     self.get_logger().info("Already received a goal shelf pose, ignoring new one until current one is processed.\n")
    
    def tf_callback(self):
        try:
            transform = self.tf_buffer.lookup_transform('base_link', 'gripper_tcp', rclpy.time.Time())
        except TransformException as e:
            self.get_logger().error(f'Could not get transform: {e}')
            return

        self.current_shelf_pose_msg.header.frame_id = 'base_link'
        self.current_shelf_pose_msg.header.stamp = self.get_clock().now().to_msg()
        self.current_shelf_pose_msg.pose.position.x = transform.transform.translation.x
        self.current_shelf_pose_msg.pose.position.y = transform.transform.translation.y
        self.current_shelf_pose_msg.pose.position.z = transform.transform.translation.z
        self.current_shelf_pose_msg.pose.orientation = transform.transform.rotation
    
    def moveit_servo_callback(self):
                
        if not self.current_shelf_pose_msg:
            self.get_logger().warning("No current shelf pose available for MoveIt Servo control.")
            return
        
        # =========================
        # safety check
        # =========================
        if self.get_coord:
            return
    
        # =========================
        # current pose
        # =========================
        current_pos = np.array([
            self.current_shelf_pose_msg.pose.position.x,
            self.current_shelf_pose_msg.pose.position.y,
            self.current_shelf_pose_msg.pose.position.z
        ])
    
        # =========================
        # target pose
        # =========================
        target_pos = np.array([
            self.goal_shelf_pose_msg.position.x,
            self.goal_shelf_pose_msg.position.y,
            self.goal_shelf_pose_msg.position.z
        ])
    
        # =========================
        # position error
        # =========================
        pos_error = target_pos - current_pos
    
        distance = np.linalg.norm(pos_error)
    
        # deadband
        if distance < self.position_deadband:
            pos_error[:] = 0.0
    
        # =========================
        # linear velocity (P control)
        # =========================
        linear_vel = self.kp_pos * pos_error
    
        # clamp linear velocity
        linear_speed = np.linalg.norm(linear_vel)
    
        if linear_speed > self.max_linear_vel:
            linear_vel = (
                linear_vel / linear_speed
            ) * self.max_linear_vel
    
        # =========================
        # quaternion rotation error
        # =========================
        target_rot = R.from_quat([
            self.goal_shelf_pose_msg.orientation.x,
            self.goal_shelf_pose_msg.orientation.y,
            self.goal_shelf_pose_msg.orientation.z,
            self.goal_shelf_pose_msg.orientation.w
        ])
    
        current_rot = R.from_quat([
            self.current_shelf_pose_msg.pose.orientation.x,
            self.current_shelf_pose_msg.pose.orientation.y,
            self.current_shelf_pose_msg.pose.orientation.z,
            self.current_shelf_pose_msg.pose.orientation.w
        ])
    
        # rotation error
        error_rot = target_rot * current_rot.inv()
    
        # quaternion shortest-path fix
        quat = error_rot.as_quat()
    
        if quat[3] < 0:
            quat *= -1.0
    
        error_rot = R.from_quat(quat)
    
        # rotation vector
        rotvec = error_rot.as_rotvec()
    
        rot_error_norm = np.linalg.norm(rotvec)
    
        # deadband
        if rot_error_norm < self.rotation_deadband:
            rotvec[:] = 0.0
    
        # =========================
        # angular velocity
        # =========================
        angular_vel = self.kp_rot * rotvec
    
        # clamp angular velocity
        angular_speed = np.linalg.norm(angular_vel)
    
        if angular_speed > self.max_angular_vel:
            angular_vel = (
                angular_vel / angular_speed
            ) * self.max_angular_vel
    
        # =========================
        # stop condition
        # =========================
        goal_reached = (
            distance < self.position_deadband and
            rot_error_norm < self.rotation_deadband
        )
    
        # =========================
        # publish twist
        # =========================
        twist_cmd = TwistStamped()
    
        twist_cmd.header.stamp = self.get_clock().now().to_msg()
        twist_cmd.header.frame_id = 'base_link'
    
        if goal_reached:
        
            twist_cmd.twist.linear.x = 0.0
            twist_cmd.twist.linear.y = 0.0
            twist_cmd.twist.linear.z = 0.0
    
            twist_cmd.twist.angular.x = 0.0
            twist_cmd.twist.angular.y = 0.0
            twist_cmd.twist.angular.z = 0.0
    
            self.get_logger().info("Goal reached.")
    
        else:
        
            twist_cmd.twist.linear.x = float(linear_vel[0])
            twist_cmd.twist.linear.y = float(linear_vel[1])
            twist_cmd.twist.linear.z = float(linear_vel[2])
    
            twist_cmd.twist.angular.x = float(angular_vel[0])
            twist_cmd.twist.angular.y = float(angular_vel[1])
            twist_cmd.twist.angular.z = float(angular_vel[2])
    
        # =========================
        # debug
        # =========================
        self.get_logger().info(
            f"\n"
            f"Position Error : {distance:.4f} m\n"
            f"Rotation Error : {rot_error_norm:.4f} rad\n"
            f"Linear Vel     : {linear_vel}\n"
            f"Angular Vel    : {angular_vel}\n"
        )
    
        # =========================
        # publish
        # =========================
        self.moveit_servo_publisher_.publish(twist_cmd)
        
    
    def servo_status_callback(self, msg):
        self.current_servo_status = msg
        # self.get_logger().info(f"Current Servo Status: {self.current_servo_status}\n")
        

def main():
    rclpy.init()

    node = ArmMoveToShelfClient()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()