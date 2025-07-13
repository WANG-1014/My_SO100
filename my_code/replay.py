import sys
import os
import time
import json
import csv
import numpy as np
from datetime import datetime
sys.path.append("..")
from scservo_sdk import *

# 路径设置
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
CALIBRATION_FILE = os.path.join(BASE_DIR, 'calibration', 'calibrate.json')
RECORD_DIR = os.path.join(BASE_DIR, 'record')

# 舵机ID
SERVO_IDS_follower = [1, 2, 3, 4, 5, 6]

# 端口号
PORT_FOLLOWER = 'COM3'
BAUDRATE = 1000000

pose_names = ["straight_pose", "rotate_pose"]

# 读取校准文件，返回主臂和从臂的两个姿态
with open(CALIBRATION_FILE, 'r', encoding='utf-8') as f:
    calib = json.load(f)
master_poses = [calib['master'][name] for name in pose_names]
follower_poses = [calib['follower'][name] for name in pose_names]
master_poses = np.array(master_poses)
follower_poses = np.array(follower_poses)

# 主从映射
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
            f = f1
        follower_angles.append(int(round(f)))
    return follower_angles

def list_record_files():
    files = [f for f in os.listdir(RECORD_DIR) if f.endswith('.csv')]
    if not files:
        print("没有找到任何录制文件！")
        return []
    print("可用录制文件：")
    for idx, f in enumerate(files):
        print(f"{idx+1}: {f}")
    return files

def load_master_sequence(filepath):
    master_seq = []
    with open(filepath, 'r', encoding='utf-8') as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            master_angles = [int(float(row[f'master_{i+1}'])) for i in range(6)]
            master_seq.append(master_angles)
    return master_seq

def main():
    files = list_record_files()
    if not files:
        return
    idx = input("请输入要回放的文件编号：").strip()
    try:
        idx = int(idx) - 1
        assert 0 <= idx < len(files)
    except:
        print("输入无效")
        return
    filepath = os.path.join(RECORD_DIR, files[idx])
    print(f"正在加载 {files[idx]} ...")
    master_seq = load_master_sequence(filepath)
    print(f"共 {len(master_seq)} 帧动作")

    # 初始化从臂端口
    portHandler_follower = PortHandler(PORT_FOLLOWER)
    packetHandler_follower = sms_sts(portHandler_follower)
    if not portHandler_follower.openPort():
        print(f"从臂串口 {PORT_FOLLOWER} 打开失败")
        return
    if not portHandler_follower.setBaudRate(BAUDRATE):
        print(f"从臂波特率设置失败")
        portHandler_follower.closePort()
        return
    print(f"从臂串口 {PORT_FOLLOWER} 打开成功")

    try:
        for idx, master_angles in enumerate(master_seq):
            follower_angles = master_to_follower(master_angles)
            for i, scs_id in enumerate(SERVO_IDS_follower):
                packetHandler_follower.SyncWritePosEx(scs_id, follower_angles[i], 0, 0)
            packetHandler_follower.groupSyncWrite.txPacket()
            packetHandler_follower.groupSyncWrite.clearParam()
            print(f"已回放第 {idx+1}/{len(master_seq)} 帧", end='\r')
            time.sleep(0.01)  # 100Hz回放
        print("\n回放完成！")
    except KeyboardInterrupt:
        print("\n回放中断")
    finally:
        portHandler_follower.closePort()
        print("串口已关闭")

if __name__ == "__main__":
    main()
