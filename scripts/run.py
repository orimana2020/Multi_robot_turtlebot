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
        self.action_client_tb1 = NavigateActionClient('tb1')
        self.action_client_tb2 = NavigateActionClient('tb2')

    def run(self):
        goal1 = np.array([0.5,0.5,1.0])
        goal2 = np.array([-0.7,-0.7,0.5])
        self.action_client_tb1.send_goal(goal2[0],goal2[1],goal2[2])
        self.action_client_tb2.send_goal(goal1[0],goal1[1],goal1[2])

           

     


def main(args=None):
    rclpy.init(args=args)
    my_node = Main_prog()
    my_node.run()
    rclpy.spin(my_node)


if __name__ == '__main__':
    main()




