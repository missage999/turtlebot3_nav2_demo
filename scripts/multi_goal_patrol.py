#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
from action_msgs.msg import GoalStatus
import time


class PatrolNode(Node):
    def __init__(self):
        super().__init__('multi_goal_patrol')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # 从ROS参数服务器获取配置参数，支持动态配置
        self.declare_parameter('timeout_duration', 30.0)
        self.declare_parameter('max_retry_count', 2)
        self.declare_parameter('retry_interval', 5.0)
        
        self.timeout_duration = self.get_parameter('timeout_duration').value
        self.max_retry_count = self.get_parameter('max_retry_count').value
        self.retry_interval = self.get_parameter('retry_interval').value
        
        self.get_logger().info(f'超时重试配置: 超时={self.timeout_duration}s, 最大重试={self.max_retry_count}, 间隔={self.retry_interval}s')
        
        # 状态管理变量
        self.current_goal_index = 0
        self.current_retry_count = 0
        self.goal_start_time = None
        self.timeout_timer = None
        self.retry_timer = None  # 专门用于重试的计时器
        self.next_goal_timer = None  # 用于延时进入下一个目标的计时器
        self.is_navigating = False    # 导航状态标志
        self.current_goal_handle = None  # 当前目标句柄
        self.navigation_failed = False  # 导航失败标志
        
        # 定义巡逻点：(x, y, yaw)
        patrol_points = [
            ( 0.500, -1.750,  0.508),
            (-0.551,  5.000,  1.500),
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

    def start_patrol(self):
        """启动巡逻（发送第一个目标）"""
        self.send_goal(0)

    def send_goal(self, goal_index, is_retry=False):
        """发送目标，支持重试机制"""
        if self.is_navigating:
            self.get_logger().warn('已有导航任务正在进行，忽略新请求')
            return
            
        self.is_navigating = True
        self.current_goal_index = goal_index
        self.goal_start_time = self.get_clock().now()
        self.current_goal_handle = None
        self.navigation_failed = False
        
        if is_retry:
            self.current_retry_count += 1
            self.get_logger().info(f'第{self.current_retry_count}次重试目标{goal_index + 1}...')
        else:
            self.current_retry_count = 0
            self.get_logger().info(f'发送目标{goal_index + 1} (x={self.goals[goal_index].pose.position.x:.3f}, y={self.goals[goal_index].pose.position.y:.3f})')
        
        # 启动超时计时器（统一的重试触发点）
        self.start_timeout_timer()
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.goals[goal_index]

        self.get_logger().info('等待动作服务器...')
        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """处理目标响应"""
        if not self.is_navigating:
            return
            
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error('目标被拒绝!')
                self.navigation_failed = True
                # 目标被拒绝时，不立即重试，等待超时计时器统一处理
                return
                
            self.get_logger().info('目标已接受.')
            self.current_goal_handle = goal_handle
            self._get_result_future = goal_handle.get_result_async()
            self._get_result_future.add_done_callback(self.get_result_callback)
        except Exception as e:
            self.get_logger().error(f'处理目标响应时出错: {e}')
            self.navigation_failed = True

    def get_result_callback(self, future):
        """处理导航结果"""
        if not self.is_navigating:
            return
            
        try:
            goal_handle = future.result()
            status = goal_handle.status
            result = goal_handle.result
            goal_num = self.current_goal_index + 1
            
            # 检查导航是否成功
            if self.is_navigation_successful(status, result):
                self.get_logger().info(f'目标{goal_num}完成.')
                print(f"\033[1;32m目标{goal_num}到达!\033[0m")
                self.is_navigating = False
                self.cancel_timeout_timer()
                self.cancel_retry_timer()
                self.current_goal_handle = None
                # 使用计时器停顿两秒后再继续下一个目标
                self.next_goal_timer = self.create_timer(2.0, self.proceed_to_next_goal)
            else:
                self.get_logger().error(f'目标{goal_num}导航失败')
                self.navigation_failed = True
                # 导航失败时，不立即重试，等待超时计时器统一处理
        except Exception as e:
            self.get_logger().error(f'处理导航结果时出错: {e}')
            self.navigation_failed = True

    def is_navigation_successful(self, status, result):
        """检查导航是否成功"""
        # 检查目标状态是否为成功
        if status == GoalStatus.STATUS_SUCCEEDED:
            return True
        else:
            # 记录失败的状态码以便调试
            status_names = {
                GoalStatus.STATUS_UNKNOWN: "未知",
                GoalStatus.STATUS_ACCEPTED: "已接受",
                GoalStatus.STATUS_EXECUTING: "执行中",
                GoalStatus.STATUS_CANCELING: "取消中",
                GoalStatus.STATUS_SUCCEEDED: "成功",
                GoalStatus.STATUS_CANCELED: "已取消",
                GoalStatus.STATUS_ABORTED: "已中止"
            }
            status_text = status_names.get(status, f"未定义状态({status})")
            self.get_logger().error(f'导航失败，状态: {status_text} ({status})')
            return False

    def proceed_to_next_goal(self):
        """继续下一个目标"""
        # 清除下一个目标的计时器
        self.cancel_next_goal_timer()
        
        next_index = self.current_goal_index + 1
        if next_index < len(self.goals):
            self.send_goal(next_index)
        else:
            self.get_logger().info('✅ 巡逻完成!')

    def feedback_callback(self, feedback_msg):
        """处理导航过程中的反馈"""
        # 可选：处理导航过程中的反馈信息
        # 例如：显示当前位置、剩余距离等
        try:
            feedback = feedback_msg.feedback
            # 可以在这里添加反馈处理逻辑
            # self.get_logger().info(f'导航反馈: 当前位置 ({feedback.current_pose.pose.position.x:.2f}, {feedback.current_pose.pose.position.y:.2f})')
        except Exception as e:
            self.get_logger().warn(f'处理导航反馈时出错: {e}')

    def start_timeout_timer(self):
        """启动超时计时器"""
        if self.timeout_timer:
            self.timeout_timer.cancel()
        
        self.timeout_timer = self.create_timer(
            self.timeout_duration, 
            self.handle_timeout
        )

    def cancel_timeout_timer(self):
        """取消超时计时器"""
        if self.timeout_timer:
            self.timeout_timer.cancel()
            self.timeout_timer = None

    def cancel_retry_timer(self):
        """取消重试计时器"""
        if self.retry_timer:
            self.retry_timer.cancel()
            self.retry_timer = None

    def cancel_next_goal_timer(self):
        """取消下一个目标的计时器"""
        if self.next_goal_timer:
            self.next_goal_timer.cancel()
            self.next_goal_timer = None

    def handle_timeout(self):
        """统一处理超时事件（唯一的重试触发点）"""
        if not self.is_navigating:
            return
            
        elapsed_time = (self.get_clock().now() - self.goal_start_time).nanoseconds / 1e9
        
        # 根据失败类型提供不同的日志信息
        if self.navigation_failed:
            self.get_logger().warn(f'目标{self.current_goal_index + 1}导航失败后超时 (已等待{elapsed_time:.1f}秒)')
        else:
            self.get_logger().warn(f'目标{self.current_goal_index + 1}通信超时 (已等待{elapsed_time:.1f}秒)')
        
        # 取消当前导航任务
        self.cancel_current_navigation()
        
        # 检查是否达到最大重试次数
        if self.current_retry_count < self.max_retry_count:
            self.get_logger().info(f'准备重试目标{self.current_goal_index + 1}...')
            # 等待重试间隔后重试
            self.retry_timer = self.create_timer(self.retry_interval, self.retry_current_goal)
        else:
            self.get_logger().error(f'目标{self.current_goal_index + 1}重试{self.max_retry_count}次后失败，跳过该目标')
            #等待一段时间后跳过当前目标
            self.retry_timer = self.create_timer(self.retry_interval, self.skip_current_goal)

    def cancel_current_navigation(self):
        """取消当前导航任务"""
        if self.current_goal_handle:
            # 如果有活跃的目标句柄，尝试取消
            try:
                self.current_goal_handle.cancel_goal_async()
                self.get_logger().info('正在取消当前导航任务...')
            except Exception as e:
                self.get_logger().warn(f'取消导航任务时出错: {e}')
        
        self.is_navigating = False
        self.current_goal_handle = None
        self.cancel_timeout_timer()
        self.cancel_retry_timer()
        self.cancel_next_goal_timer()

    def retry_current_goal(self):
        """重试当前目标"""
        # 清除重试计时器
        self.cancel_retry_timer()
        self.send_goal(self.current_goal_index, is_retry=True)

    def skip_current_goal(self):
        """跳过当前目标，继续下一个"""
        self.cancel_retry_timer()
        self.proceed_to_next_goal()


def main(args=None):
    rclpy.init(args=args)
    node = PatrolNode()
    node.start_patrol()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('巡逻被用户中断.')
    except Exception as e:
        node.get_logger().error(f'巡逻过程中发生错误: {e}')
    finally:
        # 在节点销毁前取消所有计时器
        node.cancel_timeout_timer()
        node.cancel_retry_timer()
        node.cancel_next_goal_timer()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()