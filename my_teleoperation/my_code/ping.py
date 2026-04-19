#!/usr/bin/env python
#
# *********     Ping      *********
#
##
# Ping 例程
# 设置COM_ID和Servo_ID
#

# 设置串口端口以及Ping的舵机ID         (3是从臂，7是主臂)
COM_ID = 'COM7'
Servo_ID = 1

import sys

# 导入FT舵机库
sys.path.append("../..")
from my_teleoperation.scservo_sdk import *

# 设置串口
portHandler = PortHandler(COM_ID) # (example) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

# 初始化数据包处理程序示例
# 获取协议的方法和成员
packetHandler = sms_sts(portHandler)
# 打开串口
if portHandler.openPort():
    print("成功打开串口")
else:
    print("打开串口失败")
    quit()

# 设置串口比特率
if portHandler.setBaudRate(1000000):
    print("成功设置串口比特率")
else:
    print("设置串口比特率失败")
    quit()

# Try to ping the ID:1 FTServo
# Get STServo model number
scs_model_number, scs_comm_result, scs_error = packetHandler.ping(Servo_ID)
if scs_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(scs_comm_result))
else:
    print("[ID:%03d] ping Succeeded. SCServo model number : %d" % (Servo_ID, scs_model_number))
if scs_error != 0:
    print("%s" % packetHandler.getRxPacketError(scs_error))

# 关闭串口
portHandler.closePort()
