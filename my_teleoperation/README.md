# My Robot Project 
## Initial 初始化配置
~~~ 
pip install ftservo-python-sdk
~~~
## ./my_code 文件夹是我的代码
### 一、ping.py 对应端口的对应ID的机械臂进行ping
- 需要设置COM_ID和Servo_ID
### 二、calibrate.py 机械臂校准
- 输出文件在./calibration文件夹
### 三、teleoperate.py 机械臂遥操作
- 控制主臂远程遥操作从臂
### 四、recode.py
- 输入r开始录制，输入s停止录制，输入q退出程序
- 保存文件名称可自行定义
### 五、replay.py
- 可以回复录制的轨迹
