# 起動手順(全部ラズパイ)

cd ~/MIRS2404/raspi/web
pip install -r requirements.txt
cd command
python3 web_main.py

(別のターミナルを起動)
cd mirs2404_ws
ros2 run websocket robot_control_node

(別のターミナルを起動)
ros2 run websocket raspi_receiver