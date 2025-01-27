# 起動手順(全部ラズパイ)
```bash
cd ~/MIRS2404/raspi/web 
pip install -r requirements.txt 
cd command 
python3 web_main.py 
```
別のターミナルを起動 
```bash
cd ~/mirs2404_ws
source install/setup.bash
ros2 run websocket raspi_receiver
```