#!/usr/bin/env python3
import asyncio
import websockets
import json
import configparser
import rclpy
from rclpy.node import Node
from action_msgs.srv import CancelGoal
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import SingleThreadedExecutor
import threading
import os
from ament_index_python.packages import get_package_share_directory

# configの読み込み
config_ini = configparser.ConfigParser()
config_path = os.path.join(get_package_share_directory('websocket'), 'config', 'config.ini')
config_ini.read(config_path, encoding='utf-8')

class WebSocketNode(Node):
    def __init__(self):
        super().__init__('websocket_controller')
        
        # Nav2の主要なアクションサーバーのリスト
        self.nav2_actions = [
            '/navigate_to_pose',
            '/navigate_through_poses',
            '/follow_path',
            '/follow_waypoints',
            '/assisted_teleop',
            '/backup',
            '/compute_path_through_poses',
            '/compute_path_to_pose',
            '/drive_on_heading',
            '/spin',
            '/wait'
        ]
        
        # 各アクションサーバーのキャンセルクライアントを作成
        self.cancel_clients = {}
        for action in self.nav2_actions:
            callback_group = MutuallyExclusiveCallbackGroup()
            client = self.create_client(
                CancelGoal,
                f'{action}/_action/cancel_goal',
                callback_group=callback_group
            )
            self.cancel_clients[action] = {
                'client': client,
                'callback_group': callback_group
            }

    async def cancel_all_nav2_actions(self):
        """すべてのNav2アクションをキャンセルする"""
        cancel_futures = []
        
        for action, client_info in self.cancel_clients.items():
            client = client_info['client']
            if not client.wait_for_service(timeout_sec=0.5):
                self.get_logger().warn(f'Cancel service for {action} not available')
                continue
                
            request = CancelGoal.Request()
            future = client.call_async(request)
            cancel_futures.append(future)
            
        if cancel_futures:
            # すべてのキャンセルリクエストを並行して実行
            await asyncio.get_event_loop().run_in_executor(
                None,
                self._spin_until_futures_complete,
                cancel_futures
            )
            
            # 結果をログに出力
            for action, future in zip(self.nav2_actions, cancel_futures):
                if future.done():
                    try:
                        result = future.result()
                        self.get_logger().info(f'Cancelled {action}: {result}')
                    except Exception as e:
                        self.get_logger().error(f'Failed to cancel {action}: {str(e)}')

    def _spin_until_futures_complete(self, futures):
        """複数のfutureが完了するまでスピンする"""
        executor = SingleThreadedExecutor()
        executor.add_node(self)
        for future in futures:
            executor.spin_until_future_complete(future)
        executor.shutdown()

class WebSocketServer:
    def __init__(self):
        rclpy.init()
        self.node = WebSocketNode()
        self.ros_thread = threading.Thread(target=self._ros_spin, daemon=True)
        self.ros_thread.start()

    def _ros_spin(self):
        executor = SingleThreadedExecutor()
        executor.add_node(self.node)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            self.node.destroy_node()

    async def echo(self, websocket, path):
        async for server_data in websocket:
            data = json.loads(server_data)
            print(f"受け取ったメッセージ: {data}")
            await self.control(data)

    async def control(self, data):
        if data['button'] == 'button1' and data['pressed'] == True:
            print("Nav2のアクションをすべてキャンセルします")
            await self.node.cancel_all_nav2_actions()

        if data['button'] == 'button2' and data['pressed'] == True:
            print("水出し中")
        
        if data['button'] == 'button2' and data['pressed'] == False:
            print("水出し終了")

        if data['start'] == True:
            print("走り出します")

    async def serve(self):
        host = config_ini.get('raspi_recever', 'HOST')
        port = int(config_ini.get('raspi_recever', 'PORT'))
        
        async with websockets.serve(self.echo, host, port):
            print(f"IP {host},ポート{port}で待機")
            await asyncio.Future()

def main():
    server = WebSocketServer()
    asyncio.run(server.serve())

if __name__ == "__main__":
    main()