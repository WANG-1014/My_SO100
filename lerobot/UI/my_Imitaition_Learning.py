import cv2
import os
import shutil
import subprocess
from PyQt5.QtCore import QTimer, QThread, pyqtSignal, QDir
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import *
import sys
from Imitation_Learning import Ui_MainWindow # 导入图像界面设计文件

yaocaozuo_so100_scripts_path = "/home/wang/robot/my_code/lerobot/scripts/control_robot.py"
kaishiluzhi_so100_scripts_path = "/home/wang/robot/my_code/lerobot/scripts/control_robot.py"
kaishixunlian_so100_scripts_path = "/home/wang/robot/my_code/lerobot/scripts/train.py"
kaishiyanzheng_so100_scripts_path = "/home/wang/robot/my_code/lerobot/scripts/control_robot.py"
renwuxunlianji_path = "/home/wang/.cache/huggingface/lerobot/doujiangwang"
output_path = "/home/wang/robot/my_code/outputs/train"
# stackedWidget --- 0：无机械臂；1：机械臂so-100；2：机械臂rx1-robot
# stackedWidget_2 --- 0：请选择/校准机械臂；1：请执行操作；2：开始/暂停；3：训练任务策略
# pushButton --- 0:返回；1：机器人遥操作；2：训练任务策略；3：验证任务策略；4：遥操作开始；5：遥操作停止；6：录制训练集；7：训练策略；8：返回

# 线程
class ProcessThread(QThread):
    """用于异步读取子进程输出的线程"""
    output_signal = pyqtSignal(str)  # 信号，用于发送输出内容

    def __init__(self, command):
        super().__init__()
        self.command = command
        self.process = None  # 子进程句柄
        self.running = True  # 控制线程是否继续运行

    def run(self):
        # 启动子进程，捕获 stdout 和 stderr
        self.process = subprocess.Popen(
            self.command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,  # 合并错误流到输出流
            universal_newlines=True,  # 返回字符串而非字节
            bufsize=1,  # 行缓冲
            shell=True if sys.platform == 'win32' else False
        )
        # 实时读取输出
        while self.running:
            output = self.process.stdout.readline()
            if output == '' and self.process.poll() is not None:
                break
            if output:
                self.output_signal.emit(output.strip())
        # 确保子进程正确终止
        if self.process and self.process.poll() is None:
            self.process.terminate()  # 结束子进程
            self.process.wait()  # 等待子进程完全结束

    def terminate(self):
        """安全终止线程"""
        self.running = False  # 设置标志，让循环退出
        if self.process and self.process.poll() is None:
            self.process.terminate()  # 终止子进程
            self.process.wait()  # 等待子进程结束
        self.quit()  # 退出 QThread 事件循环
        self.wait()  # 等待线程完全退出


class MyMainWindow(QMainWindow, Ui_MainWindow):  # 继承 QMainWindow类和 Ui_MainWindow界面类
    def __init__(self, parent=None):
        super(MyMainWindow, self).__init__(parent)  # 初始化父类
        self.setupUi(self)  # 继承 Ui_MainWindow 界面类
        # --------自定义参数-----------
        self.timer = self.check_interval = self.timeout_counter = self.max_checks = self.calibration_folder = self.calibration_timer = None # 基础定时器用于查找校准文件
        self.my_robot_type = 0 # 当前机械臂 --- 0：无；1：so-100；2：rx1-robot
        self.calibration_exist = 0 # 机械臂校准文件存在为：1
        self.yaocaozuo_so_100 = None # 用于so-100遥操作进程处理
        self.yaocaozuo_rx1_robot = None # 用于rx1-robot遥操作进程处理
        self.kaishiluzhi_so_100 = None # 用于so-100遥操作进程处理
        self.kaishiluzhi_rx1_robot = None # 用于rx1-robot遥操作进程处理
        self.kaishixunlian_so_100 = None # 用于so-100遥操作进程处理
        self.kaishixunlian_rx1_robot = None # 用于rx1-robot遥操作进程处理
        self.kaishiyanzheng_so_100 = None # 用于so-100验证策略处理
        self.kaishiyanzheng_rx1_robot = None # 用于rx1-robot验证策略处理
        self.ok_close = 1 # 允许关闭标志位
        self.camera1 = None # 摄像头
        self.camera2 = None # 摄像头
        self.camera_timer = None # 定时器
        self.folder_path = None

        # 配置美化
        self.textEdit.setStyleSheet("""
            QTextEdit {
                background-color: #E0E0E0;  /* 深色背景 */
                font-family: Consolas;
                font-size: 12pt;
                padding: 1px;              /* 内边距 */
            }
        """)
        self.textEdit.setReadOnly(True)
        self.textEdit_2.setStyleSheet("""
            QTextEdit {
                background-color: #E0E0E0;  /* 深色背景 */
                font-family: Consolas;
                font-size: 12pt;
                padding: 1px;              /* 内边距 */
            }
        """)
        self.textEdit_2.setReadOnly(True)
        self.listWidget.setStyleSheet("""
            QListWidget {
                font-family: Consolas;
                font-size: 12pt;
                padding: 1px;
            }
            QListWidget::item {
                padding: 2px;  /* 列表项内边距 */
            }
        """)
        self.listWidget.setAlternatingRowColors(True)  # 启用隔行变色
        self.listWidget.setSelectionMode(QListWidget.NoSelection) # 不可选择
        self.listWidget_2.setStyleSheet("""
            QListWidget {
                font-family: Consolas;
                font-size: 12pt;
                padding: 1px;
            }
            QListWidget::item {
                padding: 2px;  /* 列表项内边距 */
            }
        """)
        self.listWidget_2.setAlternatingRowColors(True)  # 启用隔行变色
        self.listWidget_2.setSelectionMode(QListWidget.NoSelection) # 不可选择
        self.label_camera1.setStyleSheet("""
            background-color: #E0E0E0;  /* 灰色 */
            padding: 1px;              /* 内边距 */
        """)
        self.label_camera2.setStyleSheet("""
            background-color: #E0E0E0;  /* 灰色 */
            padding: 1px;              /* 内边距 */
        """)
        self.textEdit_3.setStyleSheet("""
                    QTextEdit {
                        background-color: #E0E0E0;  /* 深色背景 */
                        font-family: Consolas;
                        font-size: 12pt;
                        padding: 1px;              /* 内边距 */
                    }
                """)
        self.textEdit_3.setReadOnly(True)
        self.textEdit_4.setStyleSheet("""
                    QTextEdit {
                        background-color: #E0E0E0;  /* 深色背景 */
                        font-family: Consolas;
                        font-size: 12pt;
                        padding: 1px;              /* 内边距 */
                    }
                """)
        self.textEdit_4.setReadOnly(True)
        self.textEdit_5.setStyleSheet("""
                    QTextEdit {
                        background-color: #E0E0E0;  /* 深色背景 */
                        font-family: Consolas;
                        font-size: 12pt;
                        padding: 1px;              /* 内边距 */
                    }
                """)
        self.textEdit_5.setReadOnly(True)
        self.textEdit_6.setStyleSheet("""
                    QTextEdit {
                        background-color: #E0E0E0;  /* 深色背景 */
                        font-family: Consolas;
                        font-size: 12pt;
                        padding: 1px;              /* 内边距 */
                    }
                """)
        self.textEdit_6.setReadOnly(True)
        self.renwuxunlianji_show()
        self.output_show()

        self.comboBox.currentIndexChanged.connect(self.update_line_edit)

    # 更新选择训练的训练集生成的output名
    def update_line_edit(self):
        # 根据 QComboBox 当前选中的索引设置 QLineEdit 文本
        if self.comboBox.currentIndex() == 0:
            self.lineEdit_7.setText("")
        else:
            selected_text = "act_" + self.comboBox.currentText()  # 获取当前选中的文本
            self.lineEdit_7.setText(selected_text)  # 更新 QLineEdit 的文本
    # 更新可以选择的任务训练集
    def select_renwuxunlianji(self):
        self.comboBox.clear()
        self.comboBox.addItem("")
        for index in range(self.listWidget.count()):
            item = self.listWidget.item(index).text()
            self.comboBox.addItem(item)
    # 更新可以选择的任务训练集
    def select_renwuxunlianji_2(self):
        self.comboBox_2.clear()
        self.comboBox_2.addItem("")
        for index in range(self.listWidget_2.count()):
            item = self.listWidget_2.item(index).text()
            self.comboBox_2.addItem(item)
    # 更新任务训练集
    def renwuxunlianji_show(self):
        # 训练任务集显示
        if renwuxunlianji_path:
            self.listWidget.clear()
            # 获取文件夹中的子文件夹
            dir_obj = QDir(renwuxunlianji_path)
            # 设置过滤器只显示目录
            dir_obj.setFilter(QDir.Dirs | QDir.NoDotAndDotDot)
            # 获取所有子目录
            subdirs = dir_obj.entryList()
            # 将子文件夹名称添加到文本编辑框
            for subdir in subdirs:
                self.listWidget.addItem(subdir)
    # 更新输出策略
    def output_show(self):
        # 输出策略显示
        if output_path:
            self.listWidget_2.clear()
            # 获取文件夹中的子文件夹
            dir_obj = QDir(output_path)
            # 设置过滤器只显示目录
            dir_obj.setFilter(QDir.Dirs | QDir.NoDotAndDotDot)
            # 获取所有子目录
            subdirs = dir_obj.entryList()
            # 将子文件夹名称添加到文本编辑框
            for subdir in subdirs:
                self.listWidget_2.addItem(subdir)

    # 动作 click_close 触发： ok_close == 1 才能关闭摄像头关闭程序
    def click_close(self):
        if self.ok_close == 0:
            QMessageBox.about(self, "Warming！！！",
                              """上位机还有进程没有关闭，\n请关闭进程后再结束应用！！！""")
            return
        else:
            self.close_camera()  # 关闭摄像头
            self.close()

    # 动作 actHelp 触发： 点击help弹出Help窗口
    def trigger_actHelp(self):
        QMessageBox.about(self, "Help",
                          """基于模仿学习的机器人交互-上位机 v1.0   \n""")
        return

    # 动作 so-100 触发 以及 查找校准文件
    def check_calibration_folder_so_100(self):
        self.folder_path = os.path.join(os.getcwd(), self.calibration_folder)
        if os.path.isdir(self.folder_path): # 检查是否是有效目录
            self.label_6.setText("(机械臂校准数据已就绪)")
            self.pushButton.setEnabled(True)
            self.pushButton_1.setEnabled(True)
            self.pushButton_2.setEnabled(True)
            self.pushButton_3.setEnabled(True)
            self.stackedWidget_2.setCurrentIndex(1)
            self.stackedWidget_3.setCurrentIndex(1)
            self.calibration_exist = 1
            self.calibration_timer.stop()
            return
        self.timeout_counter += 1 # 超时处理
        if self.timeout_counter >= self.max_checks:
            self.label_6.setText("(校准数据缺失，请先运行校准程序)")
            self.calibration_timer.stop()
    def trigger_so_100(self):
        self.pushButton.setEnabled(True)
        self.pushButton_1.setEnabled(False)
        self.pushButton_2.setEnabled(False)
        self.pushButton_3.setEnabled(False)
        self.calibration_exist = 0
        self.stackedWidget.setCurrentIndex(1)
        self.stackedWidget_2.setCurrentIndex(0)
        self.stackedWidget_3.setCurrentIndex(0)
        self.my_robot_type = 1
        self.label_6.setText("查找机械臂校准文件中...")
        self.check_interval = 500  # 设置检查间隔和超时（单位：毫秒） # 每0.5秒检查一次
        self.timeout_counter = 0
        self.max_checks = 5  # 2.5秒 = 5次*0.5秒
        self.calibration_folder = "./.cache/calibration/so100"  # 目标文件夹名
        self.calibration_timer = QTimer() # 启动定时器
        self.calibration_timer.timeout.connect(self.check_calibration_folder_so_100)
        self.calibration_timer.start(self.check_interval)

    # 动作 rx1_robot 触发 以及 查找校准文件
    def check_calibration_folder_rx1_robot(self):
        self.folder_path = os.path.join(os.getcwd(), self.calibration_folder)
        if os.path.isdir(self.folder_path):  # 检查是否是有效目录
            self.label_7.setText("(机械臂校准数据已就绪)")
            self.pushButton_1.setEnabled(True)
            self.pushButton_2.setEnabled(True)
            self.pushButton_3.setEnabled(True)
            self.stackedWidget_2.setCurrentIndex(1)
            self.stackedWidget_3.setCurrentIndex(1)
            self.calibration_exist = 1
            self.calibration_timer.stop()
            return
        self.timeout_counter += 1  # 超时处理
        if self.timeout_counter >= self.max_checks:
            self.label_7.setText("(校准数据缺失，请先运行校准程序)")
            self.calibration_timer.stop()
    def trigger_rx1_robot(self):
        self.pushButton.setEnabled(True)
        self.pushButton_1.setEnabled(False)
        self.pushButton_2.setEnabled(False)
        self.pushButton_3.setEnabled(False)
        self.calibration_exist = 0
        self.stackedWidget.setCurrentIndex(2)
        self.stackedWidget_2.setCurrentIndex(0)
        self.stackedWidget_3.setCurrentIndex(0)
        self.my_robot_type = 2
        self.label_7.setText("查找机械臂校准文件中...")
        self.check_interval = 500  # 设置检查间隔和超时（单位：毫秒） # 每0.5秒检查一次
        self.timeout_counter = 0
        self.max_checks = 5  # 2.5秒 = 5次*0.5秒
        self.calibration_folder = "./.cache/calibration/rx1robot"  # 目标文件夹名
        self.calibration_timer = QTimer() # 启动定时器
        self.calibration_timer.timeout.connect(self.check_calibration_folder_rx1_robot)
        self.calibration_timer.start(self.check_interval)
        return

    # 动作 click_fanhui 触发： 判断机器人类型，关闭对应的进程 self.yaocaozuo_so_100.terminate() / self.yaocaozuo_rx1_robot.terminate()
    def click_fanhui(self):
        self.pushButton.setEnabled(False)
        self.pushButton_1.setEnabled(False)
        self.pushButton_2.setEnabled(False)
        self.pushButton_3.setEnabled(False)
        self.stackedWidget.setCurrentIndex(0)
        self.my_robot_type = 0
        self.calibration_exist = 0
        self.stackedWidget_2.setCurrentIndex(0)
        self.stackedWidget_3.setCurrentIndex(0)
        if self.yaocaozuo_so_100:
            self.yaocaozuo_so_100.terminate()
        elif self.yaocaozuo_rx1_robot:
            self.yaocaozuo_rx1_robot.terminate()
        return

    # -----------------------------------------------------------------------
    # 动作 click_yaocaozuo 触发： 先检查校准文件、机械臂类型
    def click_yaocaozuo(self):
        if self.calibration_exist == 0:
            self.stackedWidget_2.setCurrentIndex(0)
            self.stackedWidget_3.setCurrentIndex(0)
        elif self.my_robot_type != 0:
            self.stackedWidget_2.setCurrentIndex(2)
            self.stackedWidget_3.setCurrentIndex(2)
        return

    # 显示摄像头： open_camera用于实时显示摄像头，update_camera_frame用于更新摄像头画面，close_camera用于关闭摄像头
    def open_camera(self):
        """打开摄像头并显示实时画面"""
        if not hasattr(self, 'camera_timer') or self.camera_timer is None:
            # 初始化摄像头定时器
            self.camera_timer = QTimer(self)
            self.camera_timer.timeout.connect(self.update_camera_frame)
        if not hasattr(self, 'camera1') or self.camera1 is None:
            # 初始化摄像头
            self.camera1 = cv2.VideoCapture(0, cv2.CAP_V4L2)
            # self.camera1 = cv2.VideoCapture(0)
            if not self.camera1.isOpened():
                QMessageBox.warning(self, "警告", "无法打开摄像头！")
                self.camera1 = None
                return False
            self.camera1.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.camera1.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            if not hasattr(self, 'camera2') or self.camera2 is None:
                # 初始化摄像头
                self.camera2 = cv2.VideoCapture(2, cv2.CAP_V4L2)
                # self.camera2 = cv2.VideoCapture(2)
                if not self.camera2.isOpened():
                    QMessageBox.warning(self, "警告", "无法打开摄像头！")
                    self.camera2 = None
                    return False
                self.camera2.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                self.camera2.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            # 开始定时器
            self.camera_timer.start(30)  # 30ms更新一帧（约33FPS）
            return True
        return False
    def update_camera_frame(self):
        """更新摄像头画面"""
        if self.camera1 and self.camera1.isOpened():
            ret, frame = self.camera1.read()
            if ret:
                # 转换颜色空间 BGR -> RGB
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                # 创建QImage并显示
                h, w, ch = frame.shape
                bytes_per_line = ch * w
                q_img = QImage(frame.data, w, h, bytes_per_line, QImage.Format_RGB888)
                # 根据当前机械臂类型显示在不同label上
                if self.my_robot_type == 1:  # so-100
                    self.label_camera1.setPixmap(QPixmap.fromImage(q_img).scaled(
                        self.label_camera1.width(),
                        self.label_camera1.height()
                    ))
                elif self.my_robot_type == 2:  # rx1-robot
                    self.label_camera1.setPixmap(QPixmap.fromImage(q_img).scaled(
                        self.label_camera1.width(),
                        self.label_camera1.height()
                    ))
        if self.camera2 and self.camera2.isOpened():
            ret, frame = self.camera2.read()
            if ret:
                # 转换颜色空间 BGR -> RGB
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                # 创建QImage并显示
                h, w, ch = frame.shape
                bytes_per_line = ch * w
                q_img = QImage(frame.data, w, h, bytes_per_line, QImage.Format_RGB888)
                # 根据当前机械臂类型显示在不同label上
                if self.my_robot_type == 1:  # so-100
                    self.label_camera2.setPixmap(QPixmap.fromImage(q_img).scaled(
                        self.label_camera2.width(),
                        self.label_camera2.height()
                    ))
                elif self.my_robot_type == 2:  # rx1-robot
                    self.label_camera2.setPixmap(QPixmap.fromImage(q_img).scaled(
                        self.label_camera2.width(),
                        self.label_camera2.height()
                    ))
    def close_camera(self):
        """关闭摄像头"""
        if hasattr(self, 'camera1') and self.camera1:
            if hasattr(self, 'camera_timer'):
                self.camera_timer.stop()
            self.camera1.release()
            self.camera1 = None
            if hasattr(self, 'camera2') and self.camera2:
                if hasattr(self, 'camera_timer'):
                    self.camera_timer.stop()
                self.camera2.release()
                self.camera2 = None
            # 清空显示
            if self.my_robot_type == 1:
                self.label_camera1.clear()
                self.label_camera2.clear()
            elif self.my_robot_type == 2:
                self.label_camera1.clear()
                self.label_camera2.clear()

    # 动作 click_yaocaozuo_kaishi 触发： 用于开启遥操作线程 self.yaocaozuo_so_100 = ProcessThread(command)
    def click_yaocaozuo_kaishi(self):
        self.pushButton_4.setEnabled(False)
        self.pushButton_4.setText("(正在进行遥操作)")
        self.pushButton.setEnabled(False)
        self.pushButton_1.setEnabled(False)
        self.pushButton_2.setEnabled(False)
        self.pushButton_3.setEnabled(False)
        if self.my_robot_type == 1:
            # self.yaocaozuo_so_100 = subprocess.Popen(['python', '../my_code/lerobot/scripts/control_robot.py', '--robot.type=so100', '--robot.cameras='+'{}', '--control.type=teleoperate'])
            self.open_camera()
            command = ['python', yaocaozuo_so100_scripts_path,
                       '--robot.type=so100',
                       '--robot.cameras={}',
                       '--control.type=teleoperate']
            self.textEdit.clear()
            # 创建并启动线程
            self.yaocaozuo_so_100 = ProcessThread(command)
            self.yaocaozuo_so_100.output_signal.connect(self.update_console)
            self.yaocaozuo_so_100.start()

            self.ok_close = 0
        elif self.my_robot_type == 2:
            self.yaocaozuo_rx1_robot = None # 这里以后写rx1的进程
            self.ok_close = 0
        return
    # 更新控制台输出
    def update_console(self, text):
        """更新控制台输出，限制最大行数"""
        max_lines = 1000  # 设置最大行数
        self.textEdit.append(text)
        # 检查并限制行数
        doc = self.textEdit.document()
        if doc.lineCount() > max_lines:
            cursor = self.textEdit.textCursor()
            cursor.movePosition(cursor.Start)
            cursor.movePosition(cursor.Down, cursor.KeepAnchor, doc.lineCount() - max_lines)
            cursor.removeSelectedText()
        self.textEdit.ensureCursorVisible()
    # 动作 click_yaocaozuo_zanting 触发： 用于结束遥操作进程
    def click_yaocaozuo_zanting(self):
        self.close_camera()  # 关闭摄像头
        if self.yaocaozuo_so_100:
            self.yaocaozuo_so_100.terminate()  # 终止子进程
            self.yaocaozuo_so_100 = None
        elif self.yaocaozuo_rx1_robot:
            self.yaocaozuo_rx1_robot.terminate()  # 终止子进程
            self.yaocaozuo_rx1_robot = None
        self.pushButton.setEnabled(True)
        self.pushButton_1.setEnabled(True)
        self.pushButton_2.setEnabled(True)
        self.pushButton_3.setEnabled(True)
        self.pushButton_4.setEnabled(True)
        self.pushButton_4.setText("开始")
        self.ok_close = 1
        return

    # -----------------------------------------------------------------------------------------------
    # 动作 click_xunliancelue 触发： 先检查校准文件、机械臂类型
    def click_xunliancelue(self):
        self.select_renwuxunlianji()
        self.renwuxunlianji_show()
        self.output_show()
        if self.calibration_exist == 0:
            self.stackedWidget_2.setCurrentIndex(0)
            self.stackedWidget_3.setCurrentIndex(0)
        elif self.my_robot_type != 0:
            self.stackedWidget_2.setCurrentIndex(3)
            self.stackedWidget_3.setCurrentIndex(3)
        return

    # 动作 click_xunliancelue_luzhixunlianji 触发：
    def click_xunliancelue_luzhixunlianji(self):
        self.renwuxunlianji_show()
        self.output_show()
        self.lineEdit.setText("")
        self.lineEdit_2.setText("")
        self.lineEdit_3.setText("")
        self.lineEdit_4.setText("")
        self.lineEdit_5.setText("")
        self.stackedWidget_4.setCurrentIndex(1)
        self.pushButton.setEnabled(False)
        self.pushButton_1.setEnabled(False)
        self.pushButton_2.setEnabled(False)
        self.pushButton_3.setEnabled(False)
        self.pushButton_6.setEnabled(False)
        self.pushButton_6.setText("(正在录制训练集)")
        self.pushButton_7.setEnabled(False)
        self.pushButton_8.setEnabled(True)
    # 动作 click_xunliancelue_xunliancelue 触发：
    def click_xunliancelue_xunliancelue(self):
        self.select_renwuxunlianji()
        self.renwuxunlianji_show()
        self.output_show()
        self.comboBox.setCurrentIndex(0)
        self.lineEdit_6.setText("100000")
        self.lineEdit_7.setText("")
        self.stackedWidget_4.setCurrentIndex(2)
        self.pushButton.setEnabled(False)
        self.pushButton_1.setEnabled(False)
        self.pushButton_2.setEnabled(False)
        self.pushButton_3.setEnabled(False)
        self.pushButton_6.setEnabled(False)
        self.pushButton_7.setEnabled(False)
        self.pushButton_7.setText("(正在训练策略)")
        self.pushButton_8.setEnabled(True)
    # 动作 click_xunliancelue_fanhui 触发：
    def click_xunliancelue_fanhui(self):
        self.stackedWidget_4.setCurrentIndex(0)
        self.pushButton.setEnabled(True)
        self.pushButton_1.setEnabled(True)
        self.pushButton_2.setEnabled(True)
        self.pushButton_3.setEnabled(True)
        self.pushButton_6.setEnabled(True)
        self.pushButton_6.setText("录制训练集")
        self.pushButton_7.setEnabled(True)
        self.pushButton_7.setText("训练策略")
        self.pushButton_8.setEnabled(False)

    # 动作 click_xunliancelue_luzhixunlianji_kaishiluzhi 触发：
    def click_xunliancelue_luzhixunlianji_kaishiluzhi(self):
        if(self.lineEdit.text().strip() == "" or
                self.lineEdit_2.text().strip() == "" or
                self.lineEdit_3.text().strip() == "" or
                self.lineEdit_4.text().strip() == "" or
                self.lineEdit_5.text().strip() == ""):
            QMessageBox.about(self, "Warming！！！",
                              """请完善训练集录制设置""")
            return
        self.pushButton.setEnabled(False)
        self.pushButton_1.setEnabled(False)
        self.pushButton_2.setEnabled(False)
        self.pushButton_3.setEnabled(False)
        self.pushButton_6.setEnabled(False)
        self.pushButton_7.setEnabled(False)
        self.pushButton_8.setEnabled(False)
        self.pushButton_9.setEnabled(False)
        self.pushButton_9.setText("(正在录制中...)")
        if self.my_robot_type == 1:
            task_name = self.lineEdit.text()
            warmup_time_s = self.lineEdit_2.text()
            episode_time_s = self.lineEdit_3.text()
            reset_time_s = self.lineEdit_4.text()
            num_episides = self.lineEdit_5.text()
            command = [
                'python', kaishiluzhi_so100_scripts_path,
                '--robot.type=so100',
                '--control.type=record',
                '--control.fps=30',
                f'--control.single_task="{task_name}"',
                f'--control.repo_id=doujiangwang/{task_name}',
                '--control.tags=["so100","tutorial"]',
                f'--control.warmup_time_s={warmup_time_s}',
                f'--control.episode_time_s={episode_time_s}',
                f'--control.reset_time_s={reset_time_s}',
                f'--control.num_episodes={num_episides}',
                '--control.push_to_hub=false'
            ]
            self.folder_path = os.path.join(renwuxunlianji_path, f"{task_name}")
            if os.path.exists(self.folder_path) and os.path.isdir(self.folder_path):
                shutil.rmtree(self.folder_path)  # 递归删除文件夹及其内容
            self.textEdit_2.clear()
            # 创建并启动线程
            self.kaishiluzhi_so_100 = ProcessThread(command)
            self.kaishiluzhi_so_100.output_signal.connect(self.update_console_2)
            self.kaishiluzhi_so_100.start()
            self.ok_close = 0
        elif self.my_robot_type == 2:
            self.kaishiluzhi_rx1_robot = None  # 这里以后写rx1的进程
            self.ok_close = 0
        return
    # 更新控制台输出
    def update_console_2(self, text):
        """更新控制台输出，限制最大行数"""
        max_lines = 1000  # 设置最大行数
        self.textEdit_2.append(text)
        # 检查是否包含 "Exiting"
        if "Exiting" in text:
            self.ok_close = 1
            self.pushButton.setEnabled(False)
            self.pushButton_1.setEnabled(False)
            self.pushButton_2.setEnabled(False)
            self.pushButton_3.setEnabled(False)
            self.pushButton_6.setEnabled(False)
            self.pushButton_7.setEnabled(False)
            self.pushButton_8.setEnabled(True)
            self.pushButton_9.setEnabled(True)
            self.pushButton_9.setText("开始录制")
            self.renwuxunlianji_show()
            self.textEdit_2.append("已完成录制，训练集已添加成功")
            self.kaishiluzhi_so_100.terminate()
            self.select_renwuxunlianji()
            self.renwuxunlianji_show()
            self.output_show()
        # 检查并限制行数
        doc = self.textEdit_2.document()
        if doc.lineCount() > max_lines:
            cursor = self.textEdit_2.textCursor()
            cursor.movePosition(cursor.Start)
            cursor.movePosition(cursor.Down, cursor.KeepAnchor, doc.lineCount() - max_lines)
            cursor.removeSelectedText()
        self.textEdit_2.ensureCursorVisible()

    def click_xunliancelue_luzhixunlianji_kaishixunlian(self):
        if self.lineEdit_7.text().strip() == "":
            QMessageBox.about(self, "Warming！！！",
                              """请选择需要训练的训练集""")
            return
        self.pushButton.setEnabled(False)
        self.pushButton_1.setEnabled(False)
        self.pushButton_2.setEnabled(False)
        self.pushButton_3.setEnabled(False)
        self.pushButton_6.setEnabled(False)
        self.pushButton_7.setEnabled(False)
        self.pushButton_8.setEnabled(False)
        self.pushButton_10.setEnabled(False)
        self.pushButton_10.setText("(正在训练中...)")
        if self.my_robot_type == 1:
            xunlian_task_name = self.comboBox.currentText()
            num_of_step = self.lineEdit_6.text()
            output_task_name = self.lineEdit_7.text()
            command = [
                'python', kaishixunlian_so100_scripts_path,
                f'--dataset.repo_id=doujiangwang/{xunlian_task_name}',
                '--policy.type=act',
                f'--output_dir=/home/wang/robot/my_code/outputs/train/{output_task_name}',
                f'--job_name={output_task_name}',
                '--device=cuda',
                '--wandb.enable=false',
                '--dataset.local_files_only=true'
            ]
            self.folder_path = os.path.join(output_path, f"{output_task_name}")
            if os.path.exists(self.folder_path) and os.path.isdir(self.folder_path):
                shutil.rmtree(self.folder_path)  # 递归删除文件夹及其内容
            self.textEdit_2.clear()
            # 创建并启动线程
            self.kaishixunlian_so_100 = ProcessThread(command)
            self.kaishixunlian_so_100.output_signal.connect(self.update_console_3)
            self.kaishixunlian_so_100.start()
            self.ok_close = 0
        elif self.my_robot_type == 2:
            self.kaishixunlian_rx1_robot = None  # 这里以后写rx1的进程
            self.ok_close = 0
        return
    # 更新控制台输出
    def update_console_3(self, text):
        """更新控制台输出，限制最大行数"""
        max_lines = 1000  # 设置最大行数
        self.textEdit_2.append(text)
        # 检查是否包含 "Exiting"
        if "End of training" in text:
            self.ok_close = 1
            self.pushButton.setEnabled(False)
            self.pushButton_1.setEnabled(False)
            self.pushButton_2.setEnabled(False)
            self.pushButton_3.setEnabled(False)
            self.pushButton_6.setEnabled(False)
            self.pushButton_7.setEnabled(False)
            self.pushButton_8.setEnabled(True)
            self.pushButton_10.setEnabled(True)
            self.pushButton_10.setText("开始训练")
            self.output_show()
            self.textEdit_2.append("已完成训练，输出策略已添加成功")
            self.kaishixunlian_so_100.terminate()
        # 检查并限制行数
        doc = self.textEdit_2.document()
        if doc.lineCount() > max_lines:
            cursor = self.textEdit_2.textCursor()
            cursor.movePosition(cursor.Start)
            cursor.movePosition(cursor.Down, cursor.KeepAnchor, doc.lineCount() - max_lines)
            cursor.removeSelectedText()
        self.textEdit_2.ensureCursorVisible()

    # --------------------------------------------------
    # 动作 click_yanzhengrenwucelue 触发： 先检查校准文件、机械臂类型
    def click_yanzhengrenwucelue(self):
        self.select_renwuxunlianji()
        self.renwuxunlianji_show()
        self.output_show()
        if self.calibration_exist == 0:
            self.stackedWidget_2.setCurrentIndex(0)
            self.stackedWidget_3.setCurrentIndex(0)
        elif self.my_robot_type != 0:
            self.stackedWidget_2.setCurrentIndex(4)
            self.stackedWidget_3.setCurrentIndex(3)
        return

    # 动作 click_yanzhengrenwucelue_yanzhengcelue
    def click_yanzhengrenwucelue_yanzhengcelue(self):
        self.lineEdit_8.clear()
        self.lineEdit_9.clear()
        self.comboBox_2.setCurrentIndex(0)
        self.renwuxunlianji_show()
        self.output_show()
        self.select_renwuxunlianji_2()
        self.stackedWidget_4.setCurrentIndex(3)
        self.pushButton.setEnabled(False)
        self.pushButton_1.setEnabled(False)
        self.pushButton_2.setEnabled(False)
        self.pushButton_3.setEnabled(False)
        self.pushButton_12.setEnabled(False)
        self.pushButton_12.setText("(正在验证策略)")
        self.pushButton_11.setEnabled(True)

    # 动作 click_yanzhengrenwucelue_fanhui
    def click_yanzhengrenwucelue_fanhui(self):
        self.stackedWidget_4.setCurrentIndex(0)
        self.pushButton.setEnabled(True)
        self.pushButton_1.setEnabled(True)
        self.pushButton_2.setEnabled(True)
        self.pushButton_3.setEnabled(True)
        self.pushButton_12.setEnabled(True)
        self.pushButton_12.setText("验证策略")
        self.pushButton_11.setEnabled(False)

    # 动作 click_yanzhengrenwucelue_yanzhengcelue_kaishi
    def click_yanzhengrenwucelue_yanzhengcelue_kaishi(self):
        if (self.comboBox_2.currentText().strip() == "" or
                self.lineEdit_8.text().strip() == "" or
                self.lineEdit_9.text().strip() == ""):
            QMessageBox.about(self, "Warming！！！",
                              """请配置好验证策略的设置""")
            return
        self.pushButton_13.setEnabled(False)
        self.pushButton_14.setEnabled(True)

        if self.my_robot_type == 1:
            name = self.comboBox_2.currentText()
            wait_time = self.lineEdit_9.text()
            total_time = self.lineEdit_8.text()
            command = [
                'python', kaishiyanzheng_so100_scripts_path,
                '--robot.type=so100',
                '--control.type=record',
                '--control.fps=30',
                f'--control.single_task="{name}"',
                f'--control.repo_id=doujiangwang/eval_{name}',
                '--control.tags=["tutorial"]',
                f'--control.warmup_time_s={wait_time}',
                f'--control.episode_time_s={total_time}',
                '--control.reset_time_s=1',
                '--control.num_episodes=1',
                '--control.push_to_hub=false',
                f'--control.policy.path=/home/wang/robot/my_code/outputs/train/{name}/checkpoints/last/pretrained_model'
            ]
            self.folder_path = os.path.join(renwuxunlianji_path, f"eval_{name}")
            if os.path.exists(self.folder_path) and os.path.isdir(self.folder_path):
                shutil.rmtree(self.folder_path)  # 递归删除文件夹及其内容
            self.textEdit_5.clear()
            # 创建并启动线程
            self.kaishiyanzheng_so_100 = ProcessThread(command)
            self.kaishiyanzheng_so_100.output_signal.connect(self.update_console_4)
            self.kaishiyanzheng_so_100.start()
            self.ok_close = 0
        elif self.my_robot_type == 2:
            self.kaishiyanzheng_rx1_robot = None  # 这里以后写rx1的进程
            self.ok_close = 0
        return

    # 更新控制台输出
    def update_console_4(self, text):
        """更新控制台输出，限制最大行数"""
        max_lines = 1000  # 设置最大行数
        self.textEdit_5.append(text)
        # 检查是否包含 "Exiting"
        if "Exiting" in text:
            self.ok_close = 1
            self.pushButton_13.setEnabled(True)
            self.pushButton_14.setEnabled(False)
            self.textEdit_5.append("已完成验证")
            self.kaishiyanzheng_so_100.terminate()
            if os.path.exists(self.folder_path) and os.path.isdir(self.folder_path):
                shutil.rmtree(self.folder_path)  # 递归删除文件夹及其内容
        # 检查并限制行数
        doc = self.textEdit_5.document()
        if doc.lineCount() > max_lines:
            cursor = self.textEdit_5.textCursor()
            cursor.movePosition(cursor.Start)
            cursor.movePosition(cursor.Down, cursor.KeepAnchor, doc.lineCount() - max_lines)
            cursor.removeSelectedText()
        self.textEdit_5.ensureCursorVisible()


    # 动作 click_yanzhengrenwucelue_yanzhengcelue_tingzhi
    def click_yanzhengrenwucelue_yanzhengcelue_tingzhi(self):
        self.pushButton_13.setEnabled(True)
        self.pushButton_14.setEnabled(False)
        self.kaishiyanzheng_so_100.terminate()
        name = self.comboBox_2.currentText()
        self.folder_path = os.path.join(renwuxunlianji_path, f"eval_{name}")
        if os.path.exists(self.folder_path) and os.path.isdir(self.folder_path):
            shutil.rmtree(self.folder_path)  # 递归删除文件夹及其内容

if __name__ == '__main__':
    app = QApplication(sys.argv)  # 在 QApplication 方法中使用，创建应用程序对象
    myWin = MyMainWindow()  # 实例化 MyMainWindow 类，创建主窗口
    myWin.show()  # 在桌面显示控件 myWin
    sys.exit(app.exec_())  # 结束进程，退出程序
