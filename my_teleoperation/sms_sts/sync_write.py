#!/usr/bin/env python
#
# *********     Sync Write Example      *********
#
#
# Available SCServo model on this example : All models using Protocol SCS
# This example is tested with a SCServo(STS/SMS), and an URT
#

import sys  # 导入sys模块，用于操作系统相关功能

sys.path.append("../..")  # 将上级目录加入模块搜索路径，便于导入scservo_sdk
from my_teleoperation.scservo_sdk import *  # 导入SCServo SDK库

# 初始化端口处理对象，指定串口号
# Windows下如"COM1"，Linux下如"/dev/ttyUSB0"
portHandler = PortHandler('/dev/ttyUSB0')  # 创建端口处理对象

# 初始化协议处理对象（PacketHandler），用于后续所有舵机通信
packetHandler = sms_sts(portHandler)  # 创建协议处理对象
    
# 打开串口
if portHandler.openPort():  # 尝试打开串口
    print("Succeeded to open the port")  # 打开成功
else:
    print("Failed to open the port")  # 打开失败
    quit()  # 退出程序

# 设置串口波特率为1000000
if portHandler.setBaudRate(1000000):  # 设置波特率
    print("Succeeded to change the baudrate")  # 设置成功
else:
    print("Failed to change the baudrate")  # 设置失败
    quit()  # 退出程序

while 1:  # 无限循环
    for scs_id in range(1, 11):  # 控制舵机ID 1~10
        # 向同步写参数区添加舵机目标位置、速度、加速度
        # 4095为目标位置，60为速度，50为加速度
        scs_addparam_result = packetHandler.SyncWritePosEx(scs_id, 4095, 60, 50)  # 添加同步写参数
        if scs_addparam_result != True:  # 检查添加是否成功
            print("[ID:%03d] groupSyncWrite addparam failed" % scs_id)  # 添加失败提示

    # 批量发送同步写指令，所有参数一次性下发
    scs_comm_result = packetHandler.groupSyncWrite.txPacket()  # 发送同步写指令
    if scs_comm_result != COMM_SUCCESS:  # 检查通信是否成功
        print("%s" % packetHandler.getTxRxResult(scs_comm_result))  # 打印错误信息

    # 清空同步写参数区，为下次写入做准备
    packetHandler.groupSyncWrite.clearParam()  # 清空参数区

    # 等待舵机运动到目标位置，时间估算公式如下
    time.sleep(((4095-0)/(60*50) + (60*50)/(50*100) + 0.05))  # 估算运动时间
    # [(P1-P0)/(V*50)] + [(V*50)/(A*100)] + 0.05
    # 其中P1-P0为位移，V为速度，A为加速度

    for scs_id in range(1, 11):  # 控制舵机ID 1~10回到0位置
        scs_addparam_result = packetHandler.SyncWritePosEx(scs_id, 0, 60, 50)  # 添加同步写参数
        if scs_addparam_result != True:  # 检查添加是否成功
            print("[ID:%03d] groupSyncWrite addparam failed" % scs_id)  # 添加失败提示

    # 批量发送同步写指令
    scs_comm_result = packetHandler.groupSyncWrite.txPacket()  # 发送同步写指令
    if scs_comm_result != COMM_SUCCESS:  # 检查通信是否成功
        print("%s" % packetHandler.getTxRxResult(scs_comm_result))  # 打印错误信息
    
    # 清空同步写参数区
    packetHandler.groupSyncWrite.clearParam()  # 清空参数区
    
    # 等待舵机运动到0位置
    time.sleep(((4095-0)/(60*50) + (60*50)/(50*100) + 0.05))  # 估算运动时间
    # 同上，估算运动时间

# 关闭串口
portHandler.closePort()  # 关闭串口
