#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from tf2_ros.buffer import Buffer
import numpy as np 


class GetPose(Node):
    def __init__(self):
        super().__init__('getpose_subscriber'+'tb1')
        self.subscription = self.create_subscription(
            TFMessage,
            '/tb1'+'/tf',
            self.get_current_pos_cb,
            10)
        self.flag1 = False
        self.flag2 = False
        # self.parent = 'map'
        # self.mid = 'odom'
        # self.final = 'base_footprint'
    def get_current_pos_cb(self, msg:TFMessage):
        
        for trans in msg.transforms:
            # if self.tf1_mat and self.tf2_mat:
            #     break
            if trans.header.frame_id == 'map' and trans.child_frame_id == 'odom':
                self.tf1_mat = self.get_translation_matrix(trans)
                self.flag1 = True
            elif trans.header.frame_id == 'odom' and trans.child_frame_id == 'base_footprint':
                self.tf2_mat = self.get_translation_matrix(trans)
                self.flag2 = True
            
        if self.flag1 and self.flag2:
            translation_matrix = np.matmul(self.tf1_mat, self.tf2_mat)
            robot_pos = np.matmul(translation_matrix, np.array([0.0,0.0,1.0]))
            print(robot_pos)

    def get_translation_matrix(self, tf:TransformStamped):
        tfx = tf.transform.translation.x
        tfy = tf.transform.translation.y
        yaw = self.euler_from_quaternion(tf.transform.rotation)

        return np.array([[np.cos(yaw), np.sin(yaw), tfx],
                        [np.sin(yaw), np.cos(yaw), tfy],
                        [0,0,1]])
        

    def euler_from_quaternion(self, quaternion):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        # return roll, pitch, yaw
        return yaw


    def quaternion_from_euler(self, roll, pitch, yaw):
        """
        Converts euler roll, pitch, yaw to quaternion (w in last place)
        quat = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)

        q = [0] * 4
        q[0] = cy * cp * cr + sy * sp * sr
        q[1] = cy * cp * sr - sy * sp * cr
        q[2] = sy * cp * sr + cy * sp * cr
        q[3] = sy * cp * cr - cy * sp * sr

        return q
      

    
    
        
def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = GetPose()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()