#!/usr/bin/env python3
import asyncio
import websockets
import json
import configparser
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from mirs_msgs.srv import SimpleCommand
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
        
        # ROSロガーの設定
        self.get_logger().info('WebSocket controller node started')

        # 水サービスクライアントの初期化
        self.water_client = None
        self.setup_water_client()

    def setup_water_client(self):
        """水サービスクライアントのセットアップ"""
        self.water_client = self.create_client(
            SimpleCommand, 
            '/water',
            callback_group=MutuallyExclusiveCallbackGroup()
        )
        # 初回接続確認（1秒待機）
        if self.water_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('水サービスの準備が完了しました')
        else:
            self.get_logger().warn('水サービスは現在利用できません')

    async def call_service_async(self, client, request):
        """サービスを非同期で呼び出す"""
        future = client.call_async(request)
        await asyncio.get_event_loop().run_in_executor(
            None,
            self._spin_until_future_complete,
            future
        )
        return future.result()

    def _spin_until_future_complete(self, future):
        """futureが完了するまでスピンする"""
        executor = SingleThreadedExecutor()
        executor.add_node(self)
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
        if data['button'] == 'button2' and data['pressed'] == True:
            print("水出し中")
            # 水サービスの呼び出し
            if self.node.water_client and self.node.water_client.wait_for_service(timeout_sec=0.1):
                request = SimpleCommand.Request()
                self.node.get_logger().info('水サービスにリクエストを送信します')
                try:
                    response = await self.node.call_service_async(self.node.water_client, request)
                    self.node.get_logger().info('水サービスからレスポンスを受信しました')
                    if response.success:
                        self.node.get_logger().info('水の供給に成功しました')
                    else:
                        self.node.get_logger().warn('水の供給に失敗しました')
                except Exception as e:
                    self.node.get_logger().error(f'水サービスの呼び出しでエラーが発生しました: {str(e)}')
            else:
                self.node.get_logger().error('水サービスは利用できません')

        if data['button'] == 'button2' and data['pressed'] == True:
            print("水出し中")
            # 水サービスの呼び出し
            if self.node.water_client and self.node.water_client.wait_for_service(timeout_sec=0.1):
                request = SimpleCommand.Request()
                try:
                    response = await self.node.call_service_async(self.node.water_client, request)
                    if response.success:
                        self.node.get_logger().info('水の供給に成功しました')
                    else:
                        self.node.get_logger().warn('水の供給に失敗しました')
                except Exception as e:
                    self.node.get_logger().error(f'水サービスの呼び出しでエラーが発生しました: {str(e)}')
            else:
                self.node.get_logger().error('水サービスは利用できません')
        
        if data['button'] == 'button2' and data['pressed'] == False:
            print("水出し終了")

        if data['start'] == True:
            print("走り出します")
            # resume_navigationサービスを呼び出し
            resume_client = self.node.create_client(Trigger, 'resume_navigation')
            if resume_client.wait_for_service(timeout_sec=1.0):
                request = Trigger.Request()
                response = await self.node.call_service_async(resume_client, request)
                if response.success:
                    self.node.get_logger().info('Navigation resumed successfully')
                else:
                    self.node.get_logger().warn('Failed to resume navigation')
            else:
                self.node.get_logger().error('Resume service not available')

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