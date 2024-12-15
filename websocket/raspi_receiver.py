#!/usr/bin/env python3
import asyncio
import websockets
import json
import configparser
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import threading
import os
from ament_index_python.packages import get_package_share_directory
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import SingleThreadedExecutor

# configの読み込み
config_ini = configparser.ConfigParser()
config_path = os.path.join(get_package_share_directory('websocket'), 'config', 'config.ini')
config_ini.read(config_path, encoding='utf-8')

class WebSocketNode(Node):
    def __init__(self):
        super().__init__('websocket_controller')
        
        # 各サービスクライアント用の個別のコールバックグループを作成
        self.pause_callback_group = MutuallyExclusiveCallbackGroup()
        self.resume_callback_group = MutuallyExclusiveCallbackGroup()
        
        # サービスクライアントの作成
        self.pause_client = self.create_client(
            Trigger, 
            'pause_robot_service',
            callback_group=self.pause_callback_group
        )
        
        self.resume_client = self.create_client(
            Trigger, 
            'resume_robot_service',
            callback_group=self.resume_callback_group
        )

    async def call_pause_service(self):
        while not self.pause_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('pause_robot_service not available, waiting...')
        
        request = Trigger.Request()
        future = self.pause_client.call_async(request)
        await asyncio.get_event_loop().run_in_executor(None, self._spin_until_future_complete, future)
        return future.result()

    async def call_resume_service(self):
        while not self.resume_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('resume_robot_service not available, waiting...')
        
        request = Trigger.Request()
        future = self.resume_client.call_async(request)
        await asyncio.get_event_loop().run_in_executor(None, self._spin_until_future_complete, future)
        return future.result()

    def _spin_until_future_complete(self, future):
        executor = SingleThreadedExecutor()
        executor.add_node(self)
        executor.spin_until_future_complete(future)
        executor.shutdown()

class WebSocketServer:
    def __init__(self):
        # ROS2ノードの初期化
        rclpy.init()
        self.node = WebSocketNode()
        
        # ROS2スピンループ用のスレッド作成と開始
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
            print("止まります")
            result = await self.node.call_pause_service()
            print(f"Pause result: {result.message}")

        if data['button'] == 'button2' and data['pressed'] == True:
            print("水出し中")
        
        if data['button'] == 'button2' and data['pressed'] == False:
            print("水出し終了")

        if data['start'] == True:
            print("走り出します")
            result = await self.node.call_resume_service()
            print(f"Resume result: {result.message}")

    async def serve(self):
        host = config_ini.get('raspi_recever', 'HOST')
        port = int(config_ini.get('raspi_recever', 'PORT'))
        
        async with websockets.serve(self.echo, host, port):
            print(f"IP {host},ポート{port}で待機")
            await asyncio.Future()  # 無限に待機

def main():
    server = WebSocketServer()
    asyncio.run(server.serve())

if __name__ == "__main__":
    main()