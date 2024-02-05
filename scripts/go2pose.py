#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from nav2_msgs.action import NavigateToPose

class NavigateActionClient(Node):

    def __init__(self, namespace):
        super().__init__('Navigate2pose'+namespace)
        self._action_client = ActionClient(self, NavigateToPose, '/'+namespace+'/navigate_to_pose')

    def send_goal(self, x,y,w):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = w


        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.current_pose))
        self.get_logger().info('Received feedback: {0}'.format(feedback.navigation_time))
        self.get_logger().info('Received feedback: {0}'.format(feedback.estimated_time_remaining))


def main(args=None):
    rclpy.init(args=args)

    action_client = NavigateActionClient('tb1')

    action_client.send_goal(-0.7,-0.7,0.5)

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()








# ros2 action send_goal
# action name:/tb1/navigate_to_pose 
# action type: nav2_msgs/action/NavigateToPose

# "pose: {header: {frame_id: map}, pose: {position: {x: 0.5, y: 0.5, z: 0.0}, orientation:{x: 0.0, y: 0.0, z: 0, w: 1.0000000}}}"