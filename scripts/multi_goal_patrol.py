#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
import time


class PatrolNode(Node):
    def __init__(self):
        super().__init__('multi_goal_patrol')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # 定义巡逻点：(x, y, yaw)
        patrol_points = [
            ( 0.500, -1.750,  0.508),
            (-0.551,  1.565,  1.500),
            ( 1.823, -0.004, -1.580),
            (-0.965,  2.014, -2.240),
            (-1.991, -0.489,  0.008),
        ]
        self.goals = [self.create_pose(x, y, yaw) for x, y, yaw in patrol_points]

    def create_pose(self, x, y, yaw):
        """创建一个 PoseStamped 消息，正确设置位置和朝向"""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        
        # 将 yaw（弧度）转换为四元数
        q = quaternion_from_euler(0.0, 0.0, yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        
        return pose

    def send_goal(self, goal_index):
        """发送第 goal_index 个目标"""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.goals[goal_index]

        self.get_logger().info(f'Waiting for action server...')
        self._action_client.wait_for_server()

        self.get_logger().info(f'Sending goal {goal_index + 1}...')
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        self.current_goal_index = goal_index

    def goal_response_callback(self, future):
        """处理目标是否被接受"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return
        self.get_logger().info('Goal accepted.')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """处理导航结果（到达/失败）"""
        result = future.result().result
        goal_num = self.current_goal_index + 1
        self.get_logger().info(f'Goal {goal_num} completed.')
        print(f"\033[1;32mGoal {goal_num} reached!\033[0m")  # 绿色高亮输出

        # 发送下一个目标
        next_index = self.current_goal_index + 1
        if next_index < len(self.goals):
            time.sleep(1.0)  # 短暂停顿
            self.send_goal(next_index)
        else:
            self.get_logger().info('✅ Patrol completed!')

    def feedback_callback(self, feedback_msg):
        """可选：处理导航过程中的反馈"""
        # feedback = feedback_msg.feedback
        # self.get_logger().info(f'Navigation feedback: {feedback}')
        pass

    def start_patrol(self):
        """启动巡逻（发送第一个目标）"""
        self.send_goal(0)


def main(args=None):
    rclpy.init(args=args)
    node = PatrolNode()
    node.start_patrol()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Patrol interrupted by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()