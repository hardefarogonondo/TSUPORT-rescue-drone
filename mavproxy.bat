cd C:/Python27/Scripts
python mavproxy.py --master tcp:192.168.0.99:6789 baudrate 57600 --out 127.0.0.1:14550 --out 127.0.0.1:14551
cmd.exe

python mavproxy.py --master /dev/ttyUSB0 --baudrate 57600 --out 127.0.0.1:14550 --out 127.0.0.1:14551
