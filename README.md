# 起動手順(全部ラズパイ)
```bash
cd ~/MIRS2404/raspi/web 
pip install -r requirements.txt 
cd command 
python3 web_main.py 
```
別のターミナルを起動 
```bash
cd mirs2404_ws
ros2 run websocket raspi_receiver
```

```
pip3 install websockets
```
localhostを使用していますが、もしPC（Nav2が動作している方）から接続する必要がある場合は、localhostではなくラズパイの実際のIPアドレスを指定する必要があります。その場合は、[raspi_recever]のHOSTを0.0.0.0（すべてのインターフェースで待ち受け）に、[socket_connection]のSEND_TO_RASPI_IPをラズパイの実際のIPアドレスに変更する必要があります。
