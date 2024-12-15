#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from rclpy.callback_groups import ReentrantCallbackGroup

class RobotControlNode(Node):
    def __init__(self):
        super().__init__('robot_control_node')
        
        # コールバックグループの作成
        self.callback_group = ReentrantCallbackGroup()
        
        # ロボットの状態を管理
        self.is_paused = False
        
        # Pauseサービスの作成
        self.pause_service = self.create_service(
            Trigger,
            'pause_robot_service',
            self.pause_callback,
            callback_group=self.callback_group
        )
        
        # Resumeサービスの作成
        self.resume_service = self.create_service(
            Trigger,
            'resume_robot_service',
            self.resume_callback,
            callback_group=self.callback_group
        )
        
        self.get_logger().info('Robot control services are ready')

    def pause_callback(self, request, response):
        """ロボットを一時停止するコールバック関数"""
        if not self.is_paused:
            self.is_paused = True
            response.success = True
            response.message = 'Robot paused successfully'
            self.get_logger().info('Robot paused')
        else:
            response.success = False
            response.message = 'Robot is already paused'
            self.get_logger().warn('Robot is already paused')
        return response

    def resume_callback(self, request, response):
        """ロボットを再開するコールバック関数"""
        if self.is_paused:
            self.is_paused = False
            response.success = True
            response.message = 'Robot resumed successfully'
            self.get_logger().info('Robot resumed')
        else:
            response.success = False
            response.message = 'Robot is already running'
            self.get_logger().warn('Robot is already running')
        return response

def main():
    rclpy.init()
    node = RobotControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()