import sys
import os
import time
import json
import csv
import numpy as np
from datetime import datetime
import threading
sys.path.append("..")
from scservo_sdk import *

# 路径设置
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
CALIBRATION_FILE = os.path.join(BASE_DIR, 'calibration', 'calibrate.json')
RECORD_DIR = os.path.join(BASE_DIR, 'record')

# 舵机ID
SERVO_IDS_master = [1, 2, 3, 4, 5, 6]
SERVO_IDS_follower = [1, 2, 3, 4, 5, 6]

# 端口号
PORT_MASTER = 'COM7'
PORT_FOLLOWER = 'COM3'

# 其他参数
BAUDRATE = 1000000
SMS_STS_PRESENT_POSITION_L = 0x38
CONTROL_FREQ = 100  # Hz

pose_names = ["straight_pose", "rotate_pose"]  # 只用这两个

# 读取校准文件，返回主臂和从臂的两个姿态
with open(CALIBRATION_FILE, 'r', encoding='utf-8') as f:
    calib = json.load(f)
master_poses = [calib['master'][name] for name in pose_names]
follower_poses = [calib['follower'][name] for name in pose_names]
# shape: (2, 6)
master_poses = np.array(master_poses)
follower_poses = np.array(follower_poses)

# 为每个舵机建立线性映射（主臂->从臂）
def master_to_follower(master_angles):
    follower_angles = []
    for i in range(6):
        m1, m2 = master_poses[0][i], master_poses[1][i]
        f1, f2 = follower_poses[0][i], follower_poses[1][i]
        m = master_angles[i]
        if m2 != m1:
            ratio = (m - m1) / (m2 - m1)
            f = f1 + ratio * (f2 - f1)
        else:
            f = f1  # 或f2，实际无影响
        follower_angles.append(int(round(f)))
    return follower_angles

def save_to_csv(recorded_data, filename):
    """保存录制数据到CSV文件"""
    os.makedirs(RECORD_DIR, exist_ok=True)
    filepath = os.path.join(RECORD_DIR, filename)
    
    with open(filepath, 'w', newline='', encoding='utf-8') as csvfile:
        writer = csv.writer(csvfile)
        # 写入表头
        header = ['timestamp']
        header.extend([f'master_{i+1}' for i in range(6)])
        header.extend([f'follower_{i+1}' for i in range(6)])
        writer.writerow(header)
        
        # 写入数据
        for row in recorded_data:
            writer.writerow(row)
    
    print(f"录制数据已保存到: {filepath}")

def main():
    # 初始化主臂端口
    portHandler_master = PortHandler(PORT_MASTER)
    packetHandler_master = sms_sts(portHandler_master)
    if not portHandler_master.openPort():
        print(f"主臂串口 {PORT_MASTER} 打开失败")
        return
    if not portHandler_master.setBaudRate(BAUDRATE):
        print(f"主臂波特率设置失败")
        portHandler_master.closePort()
        return
    print(f"主臂串口 {PORT_MASTER} 打开成功")

    # 初始化从臂端口
    portHandler_follower = PortHandler(PORT_FOLLOWER)
    packetHandler_follower = sms_sts(portHandler_follower)
    if not portHandler_follower.openPort():
        print(f"从臂串口 {PORT_FOLLOWER} 打开失败")
        portHandler_master.closePort()
        return
    if not portHandler_follower.setBaudRate(BAUDRATE):
        print(f"从臂波特率设置失败")
        portHandler_master.closePort()
        portHandler_follower.closePort()
        return
    print(f"从臂串口 {PORT_FOLLOWER} 打开成功")

    # 初始化GroupSyncRead和groupSyncWrite
    groupSyncRead = GroupSyncRead(packetHandler_master, SMS_STS_PRESENT_POSITION_L, 4)
    for scs_id in SERVO_IDS_master:
        groupSyncRead.addParam(scs_id)
    groupSyncWrite = packetHandler_follower.groupSyncWrite

    print("遥操作录制程序已启动")
    print("-----------------------------------------")
    print("输入 'r' 开始录制")
    print("输入 's' 结束录制")
    print("输入 'q' 退出程序")
    print("-----------------------------------------")
    
    # 全局状态变量
    recording = False
    recorded_data = []
    exit_program = False
    
    # 控制线程
    def control_thread():
        nonlocal recording, recorded_data, exit_program
        
        while not exit_program:
            try:
                command = input().strip().lower()
                
                if command == 'r':
                    if not recording:
                        recording = True
                        recorded_data = []
                        print("开始录制...")
                    else:
                        print("已经在录制中...")
                        
                elif command == 's':
                    if recording:
                        recording = False
                        print("录制结束，请输入保存文件名（不需要输入.csv后缀）：")
                        filename = input().strip()
                        
                        if not filename:
                            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                            filename = f"record_{timestamp}"
                        filename = filename + ".csv"
                        
                        save_to_csv(recorded_data, filename)
                        recorded_data = []
                        
                        print("-----------------------------------------")
                        print("输入 'r' 开始录制")
                        print("输入 's' 结束录制")
                        print("输入 'q' 退出程序")
                        print("-----------------------------------------")
                    else:
                        print("当前没有在录制...")
                        
                elif command == 'q':
                    exit_program = True
                    print("退出程序...")
                    break
                    
                else:
                    print("无效命令，请输入 'r', 's' 或 'q'")
                    
            except EOFError:
                break
            except KeyboardInterrupt:
                exit_program = True
                break
    
    # 启动控制线程
    control_thread_obj = threading.Thread(target=control_thread, daemon=True)
    control_thread_obj.start()
    
    try:
        while not exit_program:
            t0 = time.perf_counter()
            
            # 读取主臂角度
            comm_result = groupSyncRead.txRxPacket()
            master_angles = []
            for scs_id in SERVO_IDS_master:
                data_result, error = groupSyncRead.isAvailable(scs_id, SMS_STS_PRESENT_POSITION_L, 4)
                if data_result:
                    pos = groupSyncRead.getData(scs_id, SMS_STS_PRESENT_POSITION_L, 2)
                    master_angles.append(pos)
                else:
                    master_angles.append(0)
            
            # 计算从臂目标角度
            follower_angles = master_to_follower(master_angles)
            
            # 同步写控制从臂
            for idx, scs_id in enumerate(SERVO_IDS_follower):
                packetHandler_follower.SyncWritePosEx(scs_id, follower_angles[idx], 0, 0)
            groupSyncWrite.txPacket()
            groupSyncWrite.clearParam()
            
            # 如果正在录制，保存数据
            if recording:
                timestamp = time.time()
                row = [timestamp] + master_angles + follower_angles
                recorded_data.append(row)
                # 每100个数据点显示一次状态
                if len(recorded_data) % 100 == 0:
                    print(f"已录制 {len(recorded_data)} 个数据点...")
            
            # 精确控制100Hz
            t1 = time.perf_counter()
            dt = t1 - t0
            sleep_time = max(0, 1.0 / CONTROL_FREQ - dt)
            time.sleep(sleep_time)
            
    except KeyboardInterrupt:
        print("\n程序被中断")
    finally:
        portHandler_master.closePort()
        portHandler_follower.closePort()
        print("程序已退出")

if __name__ == "__main__":
    main()
