import sys
import os
import json
from scservo_sdk import *

sys.path.append("..")
from scservo_sdk import *  # 使用SCServo SDK库

# 保存文件位置
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
CALIBRATION_DIR = os.path.join(BASE_DIR, 'calibration')
JSON_FILE = os.path.join(CALIBRATION_DIR, 'calibrate.json')

# 舵机ID列表
SERVO_IDS_master = [1, 2, 3, 4, 5, 6]
SERVO_IDS_follower = [1, 2, 3, 4, 5, 6]

# 端口号 (COM3是从臂，COM7是主臂)
PORT_MASTER = 'COM7'
PORT_FOLLOWER = 'COM3'

# 其他参数
BAUDRATE = 1000000
PROTOCOL_VERSION = 2.0
SMS_STS_PRESENT_POSITION_L = 0x38

pose_names = ["home_pose", "straight_pose", "rotate_pose"]

def read_arm_poses(port_name, servo_ids, arm_name):
    print(f"\n========== 现在读取 {arm_name}（{port_name}）的三个位姿 ==========")
    portHandler = PortHandler(port_name)
    packetHandler = sms_sts(portHandler)
    if not portHandler.openPort():
        print(f"串口 {port_name} 打开失败")
        return None
    print(f"串口 {port_name} 打开成功")
    if not portHandler.setBaudRate(BAUDRATE):
        print(f"波特率设置失败: {port_name}")
        portHandler.closePort()
        return None
    print(f"波特率设置成功: {port_name}")
    poses = {}
    for i, pose_name in enumerate(pose_names):
        input(f"请将机械臂（{arm_name}）移动到第{i+1}个位姿（{pose_name}），然后按回车继续...")
        groupSyncRead = GroupSyncRead(packetHandler, SMS_STS_PRESENT_POSITION_L, 4)
        for scs_id in servo_ids:
            if not groupSyncRead.addParam(scs_id):
                print(f"[ID:{scs_id:03d}] groupSyncRead addparam 失败")
        comm_result = groupSyncRead.txRxPacket()
        if comm_result != COMM_SUCCESS:
            print(packetHandler.getTxRxResult(comm_result))
            portHandler.closePort()
            return None
        angles = []
        for scs_id in servo_ids:
            data_result, error = groupSyncRead.isAvailable(scs_id, SMS_STS_PRESENT_POSITION_L, 4)
            if data_result:
                pos = groupSyncRead.getData(scs_id, SMS_STS_PRESENT_POSITION_L, 2)
                print(f"[ID:{scs_id:03d}] 角度: {pos}")
                angles.append(pos)
            else:
                print(f"[ID:{scs_id:03d}] groupSyncRead getdata 失败")
                angles.append(0)
            if error != 0:
                print(packetHandler.getRxPacketError(error))
        poses[pose_name] = angles
    portHandler.closePort()
    return poses

def main():
    os.makedirs(CALIBRATION_DIR, exist_ok=True)
    if os.path.exists(JSON_FILE):
        print("!!!!!!!!!!\r\n校准文件 <./calibration/calibrate.json> 已经存在，请手动删除后再运行\r\n!!!!!!!!!!\r\n")
        return
    # 读取主臂
    master_poses = read_arm_poses(PORT_MASTER, SERVO_IDS_master, "master")
    if master_poses is None:
        return
    # 读取从臂
    follower_poses = read_arm_poses(PORT_FOLLOWER, SERVO_IDS_follower, "follower")
    if follower_poses is None:
        return
    # 保存为json
    data = {
        "master": master_poses,
        "follower": follower_poses
    }
    with open(JSON_FILE, 'w', encoding='utf-8') as f:
        json.dump(data, f, ensure_ascii=False, indent=2)
    print("主臂和从臂的三个位姿角度已保存到 <./calibration/calibrate.json>")

if __name__ == "__main__":
    main()
