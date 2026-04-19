import sys
import os
import json
import numpy as np
sys.path.append("../..")
from my_teleoperation.scservo_sdk import *

# 路径设置
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
CALIBRATION_FILE = os.path.join(BASE_DIR, 'calibration', 'calibrate.json')

# 舵机ID
SERVO_IDS_master = [1, 2, 3, 4, 5, 6]
SERVO_IDS_follower = [1, 2, 3, 4, 5, 6]

# 端口号
PORT_MASTER = 'COM7'
PORT_FOLLOWER = 'COM3'

# 其他参数
BAUDRATE = 1000000
SMS_STS_PRESENT_POSITION_L = 0x38
CONTROL_FREQ = 100  # 控制频率Hz

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

    print("开始主臂-从臂遥操作，按Ctrl+C退出...")
    try:
        while True:
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
            packetHandler_follower.groupSyncWrite.txPacket()
            packetHandler_follower.groupSyncWrite.clearParam()
            # 控制频率
            time.sleep(1.0 / CONTROL_FREQ)
    except KeyboardInterrupt:
        print("\n遥操作已停止")
    finally:
        portHandler_master.closePort()
        portHandler_follower.closePort()

if __name__ == "__main__":
    main()
