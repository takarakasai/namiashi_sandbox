#!/bin/python3

import serial
import time

ser = serial.Serial()
#ser.baudrate = 115200
ser.baudrate = 1000000
#ser.port = "/dev/ttyUSB0"
ser.port = "/dev/ttyS0"
ser.bytesize = 8
ser.parity = "N"
ser.stopbits = 1

motor_run = bytes([0x3E, 0x88, 0x01, 0x00, 0xC7])
#                    ^^    ^^    ^^    ^^
connect   = bytes([0x3E, 0x10, 0x01, 0x00, 0x4F])
#                    ^^    ^^    ^^    ^^
deg180    = bytes([0x3E, 0xA5, 0x01, 0x04, 0xE8, 0x00, 0x50, 0x46, 0x00, 0x96])
#                                                  ^^    ^^    ^^    ^^
deg90     = bytes([0x3E, 0xA5, 0x01, 0x04, 0xE8, 0x00, 0x28, 0x23, 0x00, 0x4B])
#                                                  ^^    ^^    ^^    ^^
deg0      = bytes([0x3E, 0xA5, 0x01, 0x04, 0xE8, 0x00, 0x00, 0x00, 0x00, 0x00])
#                                                  ^^    ^^    ^^    ^^
mdegp90   = bytes([0x3E, 0xA3, 0x01, 0x08, 0xEA, 0x28, 0x23, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4B])
#                                                  ^^    ^^    ^^    ^^    ^^    ^^    ^^    ^^
mdeg0     = bytes([0x3E, 0xA3, 0x01, 0x08, 0xEA, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
#                                                  ^^    ^^    ^^    ^^    ^^    ^^    ^^    ^^
mdegm90   = bytes([0x3E, 0xA3, 0x01, 0x08, 0xEA, 0xD7, 0xDC, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xAD])
#                                                  ^^    ^^    ^^    ^^    ^^    ^^    ^^    ^^

# CheckSum8 Modulo 256
def checksum(datas):
    chksum = 0x00
    for d in datas:
        chksum = (chksum + d) & 0xFF
    return chksum

def swrite(ser, bdata):
    str=''.join(f' {byte:02x}' for byte in bdata)
    # print(f'out: {str}')
    ser.write(bdata)

def mconnect(ser, id):
    head = bytes([0x3E, 0x10, id, 0x00])
    msg = head + checksum(head).to_bytes(1, 'little')
    str=''.join(f' {byte:02x}' for byte in msg)
    # print(str)
    ser.write(msg)

def mset_0deg(ser, id, mdeg):
    head = bytes([0x3E, 0x93, id, 0x00])
    msg = head + checksum(head).to_bytes(1, 'little')
    str=''.join(f' {byte:02x}' for byte in msg)
    print(str)
    ser.write(msg)

def mget_pos(ser, id):
    # time.sleep(1.0)
    # rep = ser.read_all()
    # len0 = len(rep)
    len0 = 0
    # print("rep:{}".format(rep))
    # str='--> '.join(f' {02x }' for byte in rep)
    # print(str)
    head = bytes([0x3E, 0x92, id, 0x00])
    msg = head + checksum(head).to_bytes(1, 'little')
    # str=''.join(f' {byte:02x}' for byte in msg)
    # print(str)
    ser.write(msg)
    time.sleep(0.3)

    rep_head = bytes([0x3E, 0x92, id, 0x08])
    rep_msg  = rep_head + checksum(rep_head).to_bytes(1, 'little')
    rep = ser.read_all()
    # print("rep:{}".format(rep.hex()))
    idx = rep.find(rep_msg)
    if idx == -1:
        print(" {} ->:NG : {} (len:{}->{})".format(id, rep.hex(), len0, len(rep)))
        return False, None

    data = rep[idx+len(rep_msg):idx+len(rep_msg)+8]
    val = int.from_bytes(data, 'little', signed=True) / 1000.0
    # print(" ->:{}".format(data))
    # print(" {} >>:{}".format(id, val))
    # 3e920208dab2bb0100000000006e
    return True, val
 
def mdeg_send(ser, id, mdeg):
    head = bytes([0x3E, 0xA3, id, 0x08])
    data = (int(mdeg * 1000.0)).to_bytes(8, 'little', signed=True)
    msg = head + checksum(head).to_bytes(1, 'little') + data + checksum(data).to_bytes(1, 'little')
    str=''.join(f' {byte:02x}' for byte in msg)
    print(str)
    ser.write(msg)



ser.open()

# ids = [13, 13, 13]
# ids = [10,24,11]
ids = [10, 11, 12, 13, 14, 15, 20, 21, 22, 23, 24, 25]
ratio = {
  10 : 1.0,
  11 : 1.0,
  12 : 1.47,
  13 : 1.0,
  14 : 1.0,
  15 : 1.47,
  20 : 1.0,
  21 : 1.0,
  22 : 1.47,
  23 : 1.0,
  24 : 1.0,
  25 : 1.47,
}
direction = {
  10 : +1.0,
  11 : +1.0,
  12 : +1.0,
  13 : -1.0,
  14 : +1.0,
  15 : +1.0,
  20 : +1.0,
  21 : -1.0,
  22 : -1.0,
  23 : -1.0,
  24 : -1.0,
  25 : -1.0,
}
# ids = [10, 11, 12]
# ids = [13, 14, 15]
# ids = [20, 21, 22]
# ids = [20]
# ids = [23, 24, 25]
# id = 10
for id in ids:
  mconnect(ser, id)

for id in ids:
  rep = ser.read_all()
  count = 0
  result = False
  val = None
  while not result:
      if count >= 10:
          break
      result, val = mget_pos(ser, id)
      count = count + 1

  if result:
      print(" {} >>:{} ({})".format(id, val / ratio[id] * direction[id], val))

ser.close()
