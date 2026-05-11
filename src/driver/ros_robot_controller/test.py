#!/usr/bin/env python3
# encoding: utf-8
# stm32 python sdk (STM32 Python 软件开发工具包)
import enum        # 枚举类型库
import time        # 时间库，用于延时
import copy        # 拷贝库（代码中未直接使用，可能用于扩展）
import queue       # 队列库，用于线程间安全的数据传递
import struct      # 结构体与字节串转换库，用于解析/打包二进制数据
import serial      # 串口通信库 (需要安装 pyserial)
import threading   # 多线程库，用于后台不断接收串口数据

class PacketControllerState(enum.IntEnum):
    # 串口通信协议的解析状态机枚举
    # 协议格式: 0xAA(帧头1) 0x55(帧头2) Length(长度) Function ID(功能码) Data(数据) Checksum(CRC8校验码)
    PACKET_CONTROLLER_STATE_STARTBYTE1 = 0  # 等待接收帧头第一字节 (0xAA)
    PACKET_CONTROLLER_STATE_STARTBYTE2 = 1  # 等待接收帧头第二字节 (0x55)
    PACKET_CONTROLLER_STATE_LENGTH = 2      # 等待接收数据长度
    PACKET_CONTROLLER_STATE_FUNCTION = 3    # 等待接收功能码 (Function ID)
    PACKET_CONTROLLER_STATE_ID = 4          # (保留状态，代码实际未使用)
    PACKET_CONTROLLER_STATE_DATA = 5        # 正在接收数据体
    PACKET_CONTROLLER_STATE_CHECKSUM = 6    # 等待接收并校验校验码

class PacketFunction(enum.IntEnum):
    # 可通过串口实现的控制功能 (Function ID 列表)
    PACKET_FUNC_SYS = 0        # 系统指令 (如获取电池电压)
    PACKET_FUNC_LED = 1        # LED控制
    PACKET_FUNC_BUZZER = 2     # 蜂鸣器控制
    PACKET_FUNC_MOTOR = 3      # 电机控制
    PACKET_FUNC_PWM_SERVO = 4  # PWM舵机控制, 板子上从里到外依次为1-4接口
    PACKET_FUNC_BUS_SERVO = 5  # 总线舵机控制 (串行总线舵机)
    PACKET_FUNC_KEY = 6        # 获取板载按键状态
    PACKET_FUNC_IMU = 7        # 获取IMU(惯性测量单元，陀螺仪/加速度计)数据
    PACKET_FUNC_GAMEPAD = 8    # 获取无线手柄数据
    PACKET_FUNC_SBUS = 9       # 获取航模遥控器(SBUS接收机)数据
    PACKET_FUNC_NONE = 10      # 无效/空功能

class PacketReportKeyEvents(enum.IntEnum):
    # 按键的不同触发状态枚举 (位掩码)
    KEY_EVENT_PRESSED = 0x01            # 按下
    KEY_EVENT_LONGPRESS = 0x02          # 长按
    KEY_EVENT_LONGPRESS_REPEAT = 0x04   # 长按且保持(重复触发)
    KEY_EVENT_RELEASE_FROM_LP = 0x08    # 从长按状态中释放
    KEY_EVENT_RELEASE_FROM_SP = 0x10    # 从短按状态中释放
    KEY_EVENT_CLICK = 0x20              # 单击
    KEY_EVENT_DOUBLE_CLICK= 0x40        # 双击
    KEY_EVENT_TRIPLE_CLICK = 0x80       # 三连击

#代号	对应的 C 语言类型	含义	占用字节数	取值范围大概是
#B	unsigned char	无符号单字节整数	1 个字节	0 ~ 255
#b	signed char	有符号单字节整数	1 个字节	-128 ~ 127
#H	unsigned short	无符号短整数	2 个字节	0 ~ 65535
#h	short	有符号短整数	2 个字节	-32768 ~ 32767
#f	float	单精度浮点数（带小数）	4 个字节

# 替换后的CRC8函数（与C语言Appl_Crc8Maxim完全一致）
def checksum_crc8(data):
    """
    实现与C语言Appl_Crc8Maxim一致的CRC8校验（Maxim/Dallas标准）
    :param data: 待校验的字节数据（bytes/列表）
    :return: 8位CRC校验值（0~255）
    """
    crc = 0  # 对应C代码的ZERO_INIT
    for byte in data:
        # 确保byte是8位无符号整数（兼容列表/bytes输入）
        byte = byte & 0xFF
        # 第一步：XOR当前字节
        crc ^= byte
        
        # 第二步：逐位处理8位
        for _ in range(8):
            if crc & 0x01:  # 最低位为1
                crc = (crc >> 1) ^ 0x8C  # 右移+异或0x8C（对应C代码的反射多项式）
            else:
                crc = crc >> 1  # 仅右移
            # 确保CRC始终是8位（防止Python int变长）
            crc = crc & 0xFF
    return crc & 0xFF

class SBusStatus:
    # 航模遥控器(SBUS)状态数据结构类
    def __init__(self):
        self.channels = [0] * 16;  # 16个比例通道数据
        self.channel_17 = False    # 数字通道17 (开关量)
        self.channel_18 = False    # 数字通道18 (开关量)
        self.signal_loss = True    # 信号丢失标志
        self.fail_safe = False     # 故障保护标志(失控保护)

class Board:
    # 手柄按键的位掩码字典映射，用于解析手柄按键数据包
    buttons_map = {
            'GAMEPAD_BUTTON_MASK_L2':        0x0001,
            'GAMEPAD_BUTTON_MASK_R2':        0x0002,
            'GAMEPAD_BUTTON_MASK_SELECT':    0x0004,
            'GAMEPAD_BUTTON_MASK_START':     0x0008,
            'GAMEPAD_BUTTON_MASK_L3':        0x0020,  # 左摇杆按下
            'GAMEPAD_BUTTON_MASK_R3':        0x0040,  # 右摇杆按下
            'GAMEPAD_BUTTON_MASK_CROSS':     0x0100,  # X 键
            'GAMEPAD_BUTTON_MASK_CIRCLE':    0x0200,  # O 键
            'GAMEPAD_BUTTON_MASK_SQUARE':    0x0800,  # 方块键
            'GAMEPAD_BUTTON_MASK_TRIANGLE':  0x1000,  # 三角键
            'GAMEPAD_BUTTON_MASK_L1':        0x4000,
            'GAMEPAD_BUTTON_MASK_R1':        0x8000
    }

    def __init__(self, device="/dev/ttyS0", baudrate=115200, timeout=5):
        # 初始化开发板控制对象
        self.enable_recv = False # 接收线程使能标志
        self.frame = []          # 用于暂存接收到的单个数据包
        self.recv_count = 0      # 记录当前接收到的数据长度

        # 打开串口，默认为 /dev/ttyACM0, 波特率 1,000,000
        self.port = serial.Serial(device, baudrate, timeout=timeout)
        self.state = PacketControllerState.PACKET_CONTROLLER_STATE_STARTBYTE1 # 初始化状态机
        
        # 定义线程锁，防止在读取舵机返回数据时被其他线程干扰
        self.servo_read_lock = threading.Lock()
        self.pwm_servo_read_lock = threading.Lock()
        
        # 为每种传感器/数据定义一个容量为1的队列，确保读取时获取的是最新的一帧数据
        self.sys_queue = queue.Queue(maxsize=1)
        self.bus_servo_queue = queue.Queue(maxsize=1)
        self.pwm_servo_queue = queue.Queue(maxsize=1)
        self.key_queue = queue.Queue(maxsize=1)
        self.imu_queue = queue.Queue(maxsize=1)
        self.gamepad_queue = queue.Queue(maxsize=1)
        self.sbus_queue = queue.Queue(maxsize=1)

        # 注册各功能码对应的数据处理回调函数
        self.parsers = {
            PacketFunction.PACKET_FUNC_SYS: self.packet_report_sys,
            PacketFunction.PACKET_FUNC_KEY: self.packet_report_key,
            PacketFunction.PACKET_FUNC_IMU: self.packet_report_imu,
            PacketFunction.PACKET_FUNC_GAMEPAD: self.packet_report_gamepad,
            PacketFunction.PACKET_FUNC_BUS_SERVO: self.packet_report_serial_servo,
            PacketFunction.PACKET_FUNC_SBUS: self.packet_report_sbus,
            PacketFunction.PACKET_FUNC_PWM_SERVO: self.packet_report_pwm_servo
        }

    # ---- 以下是各种数据的接收回调函数 ----
    # put_nowait 表示非阻塞存入队列，如果队列满（抛出 queue.Full 异常）则忽略，直接丢弃旧数据
    def packet_report_sys(self, data):
        try:
            self.sys_queue.put_nowait(data)
        except queue.Full:
            pass

    def packet_report_key(self, data):
        try:
            self.key_queue.put_nowait(data)
        except queue.Full:
            pass

    def packet_report_imu(self, data):
        try:
            self.imu_queue.put_nowait(data)
        except queue.Full:
            pass

    def packet_report_gamepad(self, data):
        try:
            self.gamepad_queue.put_nowait(data)
        except queue.Full:
            pass

    def packet_report_serial_servo(self, data):
        try:
            self.bus_servo_queue.put_nowait(data)
        except queue.Full:
            pass

    def packet_report_pwm_servo(self, data):
        try:
            self.pwm_servo_queue.put_nowait(data)
        except queue.Full:
            pass

    def packet_report_sbus(self, data):
        try:
            self.sbus_queue.put_nowait(data)
        except queue.Full:
            pass

    # ---- 以下是用户获取数据的接口方法 ----
    def get_battery(self):
        # 获取电池电压
        if self.enable_recv:
            try:
                data = self.sys_queue.get(block=False) # 从队列中尝试获取数据
                if data[0] == 0x04: # 0x04 为电压上报的子命令码
                    # <H 代表小端模式下的 unsigned short (2字节)
                    return struct.unpack('<H', data[1:])[0] 
                else:
                    return None
            except queue.Empty:
                return None
        else:
            print('enable reception first!') # 提示需要先开启接收线程
            return None

    def get_button(self):
        # 获取板载按键状态
        if self.enable_recv:
            try:
                data = self.key_queue.get(block=False)
                key_id = data[0] # 按键ID
                key_event = PacketReportKeyEvents(data[1]) # 按键事件
                if key_event == PacketReportKeyEvents.KEY_EVENT_CLICK:
                    return key_id, 0  # 单击返回 0
                elif key_event == PacketReportKeyEvents.KEY_EVENT_PRESSED:
                    return key_id, 1  # 按下返回 1
            except queue.Empty:
                return None
        else:
            print('enable reception first!')
            return None

    def get_imu(self):
        # 获取IMU数据 (6轴: 加速度x,y,z 角速度x,y,z)
        if self.enable_recv:
            try:
                # <6f 代表解包成 6个小端模式的浮点数(float, 每个4字节)
                return struct.unpack('<6f', self.imu_queue.get(block=False))
            except queue.Empty:
                return None
        else:
            print('enable reception first!')
            return None

    def get_gamepad(self):
        # 获取无线手柄解析数据
        if self.enable_recv:
            try:
                # 解包手柄原始数据: H(2字节无符号整数:按键), B(1字节:十字键方向), 4b(4个1字节有符号整数:摇杆)
                gamepad_data = struct.unpack("<HB4b", self.gamepad_queue.get(block=False))
                
                # 初始化摇杆和十字键轴数据阵列: 'lx', 'ly', 'rx', 'ry', 'r2', 'l2', 'hat_x', 'hat_y'
                axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                
                # 初始化按键阵列对应关系 (16个元素对应手柄的所有独立按键)
                buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] 
                
                # 遍历掩码字典，如果对应位为1，说明该键被按下
                for b in self.buttons_map:
                    if self.buttons_map[b] & gamepad_data[0]:
                        if b == 'GAMEPAD_BUTTON_MASK_R2':
                            axes[4] = 1.0     # R2线性扳机简化为按键映射
                        elif b == 'GAMEPAD_BUTTON_MASK_L2':
                            axes[5] = 1.0     # L2线性扳机简化为按键映射
                        # 记录按键状态为1
                        elif b == 'GAMEPAD_BUTTON_MASK_CROSS': buttons[0] = 1
                        elif b == 'GAMEPAD_BUTTON_MASK_CIRCLE': buttons[1] = 1
                        elif b == 'GAMEPAD_BUTTON_MASK_SQUARE': buttons[3] = 1
                        elif b == 'GAMEPAD_BUTTON_MASK_TRIANGLE': buttons[4] = 1
                        elif b == 'GAMEPAD_BUTTON_MASK_L1': buttons[6] = 1
                        elif b == 'GAMEPAD_BUTTON_MASK_R1': buttons[7] = 1
                        elif b == 'GAMEPAD_BUTTON_MASK_SELECT': buttons[10] = 1
                        elif b == 'GAMEPAD_BUTTON_MASK_START': buttons[11] = 1
               
                # 处理摇杆数据，将其归一化到 -1.0 到 1.0 的浮点数
                if gamepad_data[2] > 0: axes[0] = -gamepad_data[2] / 127
                elif gamepad_data[2] < 0: axes[0] = -gamepad_data[2] / 128

                if gamepad_data[3] > 0: axes[1] = gamepad_data[3] / 127
                elif gamepad_data[3] < 0: axes[1] = gamepad_data[3] / 128

                if gamepad_data[4] > 0: axes[2] = -gamepad_data[4] / 127
                elif gamepad_data[4] < 0: axes[2] = -gamepad_data[4] / 128

                if gamepad_data[5] > 0: axes[3] = gamepad_data[5] / 127
                elif gamepad_data[5] < 0: axes[3] = gamepad_data[5] / 128
            
                # 处理十字键(Hat)方向，依据特定的数值映射出XY方向的值
                if gamepad_data[1] == 9: axes[6] = 1.0
                elif gamepad_data[1] == 13: axes[6] = -1.0
                if gamepad_data[1] == 11: axes[7] = -1.0
                elif gamepad_data[1] == 15: axes[7] = 1.0
                
                return axes, buttons # 返回摇杆轴(浮点)和按键(0/1)列表
            except queue.Empty:
                return None
        else:
            print('enable reception first!')
            return None

    def get_sbus(self):
        # 获取SBUS航模遥控器数据
        if self.enable_recv:
            try:
                sbus_data = self.sbus_queue.get(block=False)
                status = SBusStatus()
                # 解包16个h(short整形通道数据) + 4个B(状态标志位)
                *status.channels, ch17, ch18, sig_loss, fail_safe = struct.unpack("<16hBBBB", sbus_data)
                
                # 转换状态标志为布尔值
                status.channel_17 = ch17 != 0
                status.channel_18 = ch18 != 0
                status.signal_loss = sig_loss != 0
                status.fail_safe = fail_safe != 0
                
                data = []
                if status.signal_loss:
                    # 如果丢失信号，所有通道回中(0.5)，将油门等关键通道设为0
                    data = 16 * [0.5]
                    data[4] = 0
                    data[5] = 0
                    data[6] = 0
                    data[7] = 0
                else:
                    # 将SBUS原始数据(通常在192-1792之间)归一化为 0.0 到 1.0
                    for i in status.channels:
                        data.append((i - 192)/(1792 - 192))
                return data
            except queue.Empty:
                return None
        else:
            print('enable reception first!')
            return None

    # ---- 以下是通信发送核心逻辑 ----
    def buf_write(self, func, data):
        # 通用数据包封装发送函数
        buf = [0xAA, 0x55, int(func)] # 帧头1，帧头2，功能码
        buf.append(len(data))         # 写入数据长度
        buf.extend(data)              # 写入实际数据
        buf.append(checksum_crc8(bytes(buf[2:]))) # 计算并写入功能码+长度+数据的 CRC8 校验值
        self.port.write(buf)          # 通过串口发送出去

    # ---- 以下是开发板各硬件的控制命令 ----
    def set_led(self, on_time, off_time, repeat=1, led_id=1):
        # 控制板载LED闪烁 (亮的时间秒，灭的时间秒，重复次数，LED编号)
        on_time = int(on_time*1000) # 转换为毫秒
        off_time = int(off_time*1000)
        self.buf_write(PacketFunction.PACKET_FUNC_LED, struct.pack("<BHHH", led_id, on_time, off_time, repeat))

    def set_buzzer(self, freq, on_time, off_time, repeat=1):
        # 控制蜂鸣器发声 (频率Hz，响时间秒，停时间秒，重复次数)
        on_time = int(on_time*1000)
        off_time = int(off_time*1000)
        self.buf_write(PacketFunction.PACKET_FUNC_BUZZER, struct.pack("<HHHH", freq, on_time, off_time, repeat))

    def set_motor_speed(self, speeds):
        # 设置电机速度, speeds 是一个二维列表如: [[电机ID1, 速度1], [电机ID2, 速度2]]
        data = [0x01, len(speeds)] # 0x01为控制子命令，后面跟要控制的电机数量
        for i in speeds:
            # 电机ID通常习惯1开始，底层接收从0开始，所以 i[0]-1。速度为浮点数
            data.extend(struct.pack("<Bf", int(i[0]), float(i[1])))
        self.buf_write(PacketFunction.PACKET_FUNC_MOTOR, data)

    # ---- PWM 舵机相关操作 ----
    def pwm_servo_set_position(self, duration, positions):
        # 控制PWM舵机转动到指定位置 (运行时间秒，位置二维列表 [[ID, 脉宽], ...])
        duration = int(duration * 1000) # 转毫秒
        data = [0x01, duration & 0xFF, 0xFF & (duration >> 8), len(positions)] # 命令码，时间低位，时间高位，舵机数量
        for i in positions:
            data.extend(struct.pack("<BH", i[0], i[1])) # 打包舵机ID和脉宽值(通常500-2500)
        self.buf_write(PacketFunction.PACKET_FUNC_PWM_SERVO, data)
    
    def pwm_servo_set_offset(self, servo_id, offset):
        # 设置PWM舵机偏差值(修正零位)
        data = struct.pack("<BBb", 0x07, servo_id, int(offset))
        self.buf_write(PacketFunction.PACKET_FUNC_PWM_SERVO, data)

    def pwm_servo_read_and_unpack(self, servo_id, cmd, unpack):
        # 通用的PWM舵机读取逻辑 (附带线程锁以保证收发对应)
        with self.servo_read_lock: # 加锁
            self.buf_write(PacketFunction.PACKET_FUNC_PWM_SERVO, [cmd, servo_id]) # 发送读取请求
            data = self.pwm_servo_queue.get(block=True) # 阻塞等待返回数据
            servo_id, cmd, info = struct.unpack(unpack, data) # 按指定的解包格式解析
            return info

    def pwm_servo_read_offset(self, servo_id):
        # 读取PWM舵机偏差
        return self.pwm_servo_read_and_unpack(servo_id, 0x09, "<BBb")

    def pwm_servo_read_position(self, servo_id):
        # 读取PWM舵机当前位置脉宽
        return self.pwm_servo_read_and_unpack(servo_id, 0x05, "<BBH")

    # ---- 串行总线舵机(Bus Servo)相关操作 ----
    def bus_servo_enable_torque(self, servo_id, enable):
        # 使能/卸载 总线舵机扭矩 (上电/掉电)
        if enable:
            data = struct.pack("<BB", 0x0B, servo_id)
        else:
            data = struct.pack("<BB", 0x0C, servo_id)
        self.buf_write(PacketFunction.PACKET_FUNC_BUS_SERVO, data)
        time.sleep(0.02) # 等待执行完毕

    def bus_servo_set_id(self, servo_id_now, servo_id_new):
        # 修改总线舵机ID (原ID, 新ID)
        data = struct.pack("<BBB", 0x10, servo_id_now, servo_id_new)
        self.buf_write(PacketFunction.PACKET_FUNC_BUS_SERVO, data)
        time.sleep(0.02)

    def bus_servo_set_offset(self, servo_id, offset):
        # 设置总线舵机中位偏差
        data = struct.pack("<BBb", 0x20, servo_id, int(offset))
        self.buf_write(PacketFunction.PACKET_FUNC_BUS_SERVO, data)
        time.sleep(0.02)

    def bus_servo_save_offset(self, servo_id):
        # 保存偏差到总线舵机的内部Flash中 (掉电不丢失)
        data = struct.pack("<BB", 0x24, servo_id)
        self.buf_write(PacketFunction.PACKET_FUNC_BUS_SERVO, data)
        time.sleep(0.02)

    def bus_servo_set_angle_limit(self, servo_id, limit):
        # 设置总线舵机旋转角度限制 (limit为包含最小最大角度的列表)
        data = struct.pack("<BBHH", 0x30, servo_id, int(limit[0]), int(limit[1]))
        self.buf_write(PacketFunction.PACKET_FUNC_BUS_SERVO, data)
        time.sleep(0.02)

    def bus_servo_set_vin_limit(self, servo_id, limit):
        # 设置总线舵机输入电压报警限制
        data = struct.pack("<BBHH", 0x34, servo_id, int(limit[0]), int(limit[1]))
        self.buf_write(PacketFunction.PACKET_FUNC_BUS_SERVO, data)
        time.sleep(0.02)

    def bus_servo_set_temp_limit(self, servo_id, limit):
        # 设置总线舵机内部温度报警限制
        data = struct.pack("<BBb", 0x38, servo_id, int(limit))
        self.buf_write(PacketFunction.PACKET_FUNC_BUS_SERVO, data)
        time.sleep(0.02)

    def bus_servo_stop(self, servo_id):
        # 急停指定的总线舵机，servo_id 为舵机ID列表
        data = [0x03, len(servo_id)] 
        data.extend(struct.pack("<"+'B'*len(servo_id), *servo_id))
        self.buf_write(PacketFunction.PACKET_FUNC_BUS_SERVO, data)

    def bus_servo_set_position(self, duration, positions):
        # 控制多个总线舵机运动到指定位置 (运动时间秒，[[ID1, 位置1], [ID2, 位置2]])
        duration = int(duration * 1000)
        data = [0x01, duration & 0xFF, 0xFF & (duration >> 8), len(positions)]
        for i in positions:
            data.extend(struct.pack("<BH", i[0], i[1]))
        self.buf_write(PacketFunction.PACKET_FUNC_BUS_SERVO, data)

    def bus_servo_read_and_unpack(self, servo_id, cmd, unpack):
        # 通用的总线舵机参数读取与解包逻辑
        with self.servo_read_lock:
            self.buf_write(PacketFunction.PACKET_FUNC_BUS_SERVO, [cmd, servo_id])
            data = self.bus_servo_queue.get(block=True)
            # 总线舵机的返回数据多了一个 success 标志位
            servo_id, cmd, success, *info = struct.unpack(unpack, data)
            if success == 0: # 0 表示通信读取成功
                return info

    # 以下均为总线舵机各种参数的读取封装
    def bus_servo_read_id(self, servo_id=254):
        # 默认向 254(广播ID)发送查询命令，获取当前连接舵机的真实ID
        return self.bus_servo_read_and_unpack(servo_id, 0x12, "<BBbB")

    def bus_servo_read_offset(self, servo_id):
        return self.bus_servo_read_and_unpack(servo_id, 0x22, "<BBbb")
    
    def bus_servo_read_position(self, servo_id):
        return self.bus_servo_read_and_unpack(servo_id, 0x05, "<BBbh")

    def bus_servo_read_vin(self, servo_id):
        return self.bus_servo_read_and_unpack(servo_id, 0x07, "<BBbH")
    
    def bus_servo_read_temp(self, servo_id):
        return self.bus_servo_read_and_unpack(servo_id, 0x09, "<BBbB")

    def bus_servo_read_temp_limit(self, servo_id):
        return self.bus_servo_read_and_unpack(servo_id, 0x3A, "<BBbB")

    def bus_servo_read_angle_limit(self, servo_id):
        return self.bus_servo_read_and_unpack(servo_id, 0x32, "<BBb2H")

    def bus_servo_read_vin_limit(self, servo_id):
        return self.bus_servo_read_and_unpack(servo_id, 0x36, "<BBb2H")

    def bus_servo_read_torque_state(self, servo_id):
        return self.bus_servo_read_and_unpack(servo_id, 0x0D, "<BBbb")

    # ---- 串口数据后台接收与解析任务 ----
    def enable_reception(self):
        # 开启接收功能，启动一个守护线程后台运行 recv_task
        self.enable_recv = True
        threading.Thread(target=self.recv_task, daemon=True).start()

    def recv_task(self):
        # 后台死循环：不断从串口读取数据，并通过状态机解析数据包
        while self.enable_recv:
            recv_data = self.port.read() # 读取数据（利用了 serial 初始化时的 timeout 配置）
            if recv_data:
                for dat in recv_data:
                    # 状态机：寻找第一个帧头 0xAA
                    if self.state == PacketControllerState.PACKET_CONTROLLER_STATE_STARTBYTE1:
                        if dat == 0xAA:
                            self.state = PacketControllerState.PACKET_CONTROLLER_STATE_STARTBYTE2
                        continue
                    # 状态机：寻找第二个帧头 0x55
                    elif self.state == PacketControllerState.PACKET_CONTROLLER_STATE_STARTBYTE2:
                        if dat == 0x55:
                            self.state = PacketControllerState.PACKET_CONTROLLER_STATE_FUNCTION
                        else:
                            self.state = PacketControllerState.PACKET_CONTROLLER_STATE_STARTBYTE1 # 头错误，重置状态
                        continue
                    # 状态机：记录功能码 Function ID
                    elif self.state == PacketControllerState.PACKET_CONTROLLER_STATE_FUNCTION:
                        if dat < int(PacketFunction.PACKET_FUNC_NONE): # 校验功能码合法性
                            self.frame = [dat, 0] # 记录功能码，准备记录长度
                            self.state = PacketControllerState.PACKET_CONTROLLER_STATE_LENGTH
                        else:
                            self.frame = []
                            self.state = PacketControllerState.PACKET_CONTROLLER_STATE_STARTBYTE1 # 错误功能码，重置
                        continue
                    # 状态机：记录数据长度
                    elif self.state == PacketControllerState.PACKET_CONTROLLER_STATE_LENGTH:
                        self.frame[1] = dat
                        self.recv_count = 0
                        if dat == 0:
                            self.state = PacketControllerState.PACKET_CONTROLLER_STATE_CHECKSUM # 若无数据，直接验证校验码
                        else:
                            self.state = PacketControllerState.PACKET_CONTROLLER_STATE_DATA # 进入数据接收状态
                        continue
                    # 状态机：接收数据体
                    elif self.state == PacketControllerState.PACKET_CONTROLLER_STATE_DATA:
                        self.frame.append(dat)
                        self.recv_count += 1
                        if self.recv_count >= self.frame[1]: # 数据接收完毕
                            self.state = PacketControllerState.PACKET_CONTROLLER_STATE_CHECKSUM
                        continue
                    # 状态机：接收校验码并验证
                    elif self.state == PacketControllerState.PACKET_CONTROLLER_STATE_CHECKSUM:
                        crc8 = checksum_crc8(bytes(self.frame)) # 计算本地校验和 (包含功能码、长度、数据)
                        if crc8 == dat: # 校验通过
                            func = PacketFunction(self.frame[0]) # 获取功能码
                            data = bytes(self.frame[2:])         # 切片出纯数据体
                            if func in self.parsers:
                                self.parsers[func](data)         # 调用注册好的回调函数放入相应队列
                        else:
                            print("校验失败")
                        # 无论校验成功与否，一帧处理完毕，重置状态机寻找下一个帧头
                        self.state = PacketControllerState.PACKET_CONTROLLER_STATE_STARTBYTE1
                        continue
        # 退出循环后关闭串口
        self.port.close()
        print("END...")

# ---- 以下是功能测试函数(供演示/调试用) ----
def bus_servo_test(board):
    # 总线舵机测试例程
    board.bus_servo_set_position(1, [[1, 500], [2, 500]]) # 1号和2号舵机用1秒走到位置500
    time.sleep(1)
    board.bus_servo_set_position(2, [[1, 0], [2, 0]])     # 用2秒走到位置0
    time.sleep(1)
    board.bus_servo_stop([1, 2])                          # 停止运动
    time.sleep(1)
    
    servo_id = 1
    board.bus_servo_set_id(254, servo_id) # 将连在板子上的任意ID的舵机修改为ID=1
    servo_id = board.bus_servo_read_id()  # 读取测试
    if servo_id is not None:
        servo_id = servo_id[0]
        
        # 测试各类参数设置与读取
        offset_set = -10
        board.bus_servo_set_offset(servo_id, offset_set)
        board.bus_servo_save_offset(servo_id)
        
        vin_l, vin_h = 4500, 14500
        board.bus_servo_set_vin_limit(servo_id, [vin_l, vin_h])

        temp_limit = 85
        board.bus_servo_set_temp_limit(servo_id, temp_limit)

        angle_l, angle_h = 0, 1000
        board.bus_servo_set_angle_limit(servo_id, [angle_l, angle_h])
        
        board.bus_servo_enable_torque(servo_id, 1) # 使能扭矩

        # 打印读取的各种状态
        print('id:', board.bus_servo_read_id(servo_id))
        print('offset:', board.bus_servo_read_offset(servo_id), offset_set)
        print('vin:', board.bus_servo_read_vin(servo_id))
        print('temp:', board.bus_servo_read_temp(servo_id))
        print('position:', board.bus_servo_read_position(servo_id))
        print('angle_limit:', board.bus_servo_read_angle_limit(servo_id), [angle_l, angle_h])
        print('vin_limit:', board.bus_servo_read_vin_limit(servo_id), [vin_l, vin_h])
        print('temp_limit:', board.bus_servo_read_temp_limit(servo_id), temp_limit)
        print('torque_state:', board.bus_servo_read_torque_state(servo_id))

def pwm_servo_test(board):
    # PWM舵机测试例程
    servo_id = 1
    board.pwm_servo_set_position(0.5, [[servo_id, 1500]]) # 0.5秒走到1500(中位)
    board.pwm_servo_set_offset(servo_id, 0)
    print('offset:', board.pwm_servo_read_offset(servo_id))
    print('position:', board.pwm_servo_read_position(servo_id))


def set_motor(speed_right, speed_left, max_speed):
    # 电机控制测试例程，正值为正转，负值为反
    speed_right_real = max_speed * speed_right
    speed_left_real = max_speed * speed_left * (-1)
    board = Board()              # 实例化底层控制对象
    board.set_motor_speed([[1, speed_right_real], [2, speed_left_real]])


# 主程序执行入口
if __name__ == "__main__":
    while True:
    	print("print(w:front,s:behind,a:left,d:right,q:quit):")
    	word = input()
    	if word == 'w':
    		set_motor(0.1, 0.1, 100)
    	elif word == 's':
    		set_motor(-0.1, -0.1, 100)
    	elif word == 'a':
    		set_motor(-0.1, 0.1, 100)
    	elif word == 'd':
    		set_motor(0.1, -0.1, 100)
    	elif word == 'q':
    		set_motor(0, 0, 100)
    		break
        
        



    
