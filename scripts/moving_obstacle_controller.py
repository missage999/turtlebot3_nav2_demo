#!/usr/bin/env python3
# scripts/moving_obstacle_controller.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MovingObstacleController(Node):
    def __init__(self):
        super().__init__('moving_obstacle_controller')
        self.publisher_ = self.create_publisher(Twist, '/moving_obstacle/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.move_pattern)  # 10 Hz
        
        # 新增：就绪状态管理和等待计数
        self.ready_to_move = False
        self.wait_cycle = 0
        self.get_logger().info('Controller started. Waiting for Gazebo plugin connection...')

    def move_pattern(self):
        # 1. 等待阶段：检查是否有订阅者
        if not self.ready_to_move:
            sub_count = self.publisher_.get_subscription_count()
            if sub_count > 0:
                self.get_logger().info(f'Plugin connected! Starting motion '
                                       f'(elapsed: {self.wait_cycle * 0.1:.1f}s).')
                self.ready_to_move = True
                # 关键：重置开始时间！排除等待耗时
                self.start_time = self.get_clock().now()
            else:
                # 可选：每隔一段时间打印日志，避免刷屏
                self.wait_cycle += 1
                if self.wait_cycle % 50 == 0:  # 每5秒提示一次
                    self.get_logger().info('Still waiting for plugin subscription...')
                return  # 不发布任何指令

        # 2. 正常运行阶段：原逻辑不变
        msg = Twist()
        current_time = self.get_clock().now()
        elapsed = (current_time - self.start_time).nanoseconds / 1e9
        cycle = elapsed % 40.0

        if cycle < 20.0:
            msg.linear.x = 0.2
        else:
            msg.linear.x = -0.2

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MovingObstacleController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 停止小车
        node.publisher_.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()