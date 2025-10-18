#!/bin/python3

import serial

# シリアルポートの設定
print("open")
ser = serial.Serial(
    #port='/dev/ttyUSB0',  # 使用するポートを指定してください（例: COM3, /dev/ttyUSB0）
    #port='/dev/ttyS0',  # 使用するポートを指定してください（例: COM3, /dev/ttyUSB0）
    port='/dev/ttyAMA4',  # 使用するポートを指定してください（例: COM3, /dev/ttyUSB0）
    baudrate=115200,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    timeout=1  # 読み取りタイムアウトを1秒に設定
)
print("opended")

try:
  while True:
    if ser.in_waiting > 0:
      data = ser.read(ser.in_waiting)
      print(data)
except KeyboardInterrupt:
  print("通信を終了します")
finally:
  ser.close()
