#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from go2pose import NavigateActionClient
from get_current_position import GetPose
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np


class Main_prog(Node):
    def __init__(self):
        super().__init__('main_prog')
        
        
        # get tb1 position
        self.sub_tb1_amcl = self.create_subscription(PoseWithCovarianceStamped, '/tb1/amcl_pose', self.get_current_pos_tb1_cb, 1)
        self.sub_tb1_amcl  # prevent unused variable warning
        self.tb1_x = None
        self.tb1_y = None
        self.tb1_orientation = None

        self.action_client_tb1 = NavigateActionClient('tb1')
        self.action_client_tb2 = NavigateActionClient('tb2')

        # get tb2 position
        self.sub_tb2_amcl = self.create_subscription(PoseWithCovarianceStamped, '/tb2/amcl_pose', self.get_current_pos_tb2_cb, 1)
        self.sub_tb2_amcl  # prevent unused variable warning
        self.tb2_x = None
        self.tb2_y = None
        self.tb2_orientation = None

    def get_current_pos_tb1_cb(self, msg:PoseWithCovarianceStamped):
        if msg.pose.pose.position.x is not None:
            self.tb1_x = msg.pose.pose.position.x
            self.tb1_y = msg.pose.pose.position.y
            self.tb1_orientation = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
            self.get_logger().info('read')
      
    def get_current_pos_tb2_cb(self, msg:PoseWithCovarianceStamped):
        self.tb2_x = msg.pose.pose.position.x
        self.tb2_y = msg.pose.pose.position.y
        self.tb2_orientation = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
    
        

    def run(self):
        goal2 = np.array([0.5,0.5,1.0])
        goal1 = np.array([-0.7,-0.7,0.5])
        tb1_goal = goal1
        i=0
        # self.action_client_tb1.send_goal(tb1_goal[0],tb1_goal[1],tb1_goal[2])
        for _ in range(5):
            try:
                tb1_dist2goal = ((tb1_goal[0] - self.tb1_x)**2 + (tb1_goal[1] - self.tb1_y)**2)**0.5
                if tb1_dist2goal < 0.5:
                    if i==0:
                        tb1_goal = goal2
                        i=1
                        self.action_client_tb1.send_goal(tb1_goal[0],tb1_goal[1],tb1_goal[2])
                    elif i==1:
                        tb1_goal=goal1
                        i=0
                        self.action_client_tb1.send_goal(tb1_goal[0],tb1_goal[1],tb1_goal[2])
            except:
                pass
           

            # self.action_client_tb1.send_goal(tb1_goal[0],tb1_goal[1],tb1_goal[2])
        # self.action_client_tb2.send_goal()

        # timer_period = 0.5  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)
    
    # def timer_callback(self):
    #     print(self.msg)


def main(args=None):
    rclpy.init(args=args)
    my_node = Main_prog()
    my_node.run()
    rclpy.spin(my_node)


if __name__ == '__main__':
    main()




