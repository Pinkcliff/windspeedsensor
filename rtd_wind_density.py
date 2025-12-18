import socket
import time
from typing import List, Dict, Optional
from Refrigerant import AIR
from kalman_filter import create_wind_speed_filter
import threading
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import numpy as np


# --------------------------
# 核心工具函数：Modbus RTU帧处理（不变）
# --------------------------
def modbus_crc(data: List[int]) -> List[int]:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    return [crc & 0xFF, (crc >> 8) & 0xFF]


def build_rtu_request(slave_addr: int, start_reg: int, reg_count: int, func_code: int = 0x04) -> bytes:
    # 支持0x03（保持寄存器）和0x04（输入寄存器）切换
    frame = [
        slave_addr,
        func_code,
        (start_reg >> 8) & 0xFF,
        start_reg & 0xFF,
        (reg_count >> 8) & 0xFF,
        reg_count & 0xFF
    ]
    crc = modbus_crc(frame)
    frame.extend(crc)
    return bytearray(frame)


def parse_rtu_response(response_bytes: bytes) -> Dict:
    response = list(response_bytes)
    if len(response) < 4:
        return {"error": "响应帧过短"}

    slave_addr = response[0]
    func_code = response[1]
    data = response[2:-2]
    received_crc = response[-2:]

    calculated_crc = modbus_crc(response[:-2])
    if received_crc != calculated_crc:
        return {"error": f"CRC校验失败（接收: {received_crc}，计算: {calculated_crc}）"}

    if func_code in [0x03, 0x04]:
        if len(data) < 1:
            return {"error": f"功能码{func_code:02X}响应数据为空"}
        byte_count = data[0]
        registers = []
        for i in range(1, len(data), 2):
            if i + 1 > len(data):
                break
            reg_value = (data[i] << 8) | data[i + 1]
            registers.append(reg_value)
        return {
            "slave_addr": slave_addr,
            "func_code": func_code,
            "registers": registers,
            "valid": True
        }
    else:
        return {"error": f"不支持的功能码：0x{func_code:02X}"}


# --------------------------
# 数据共享类
# --------------------------
class SharedData:
    def __init__(self):
        # 模拟量传感器数据
        self.analog_temp = 0.0      # 温度(℃)
        self.analog_pressure = 0.0  # 压力(kPa)
        self.analog_humidity = 0.0  # 湿度(%)
        self.wind_speeds = [0.0] * 4  # 4路风速(m/s)
        self.wind_speeds_raw = [0.0] * 4  # 4路原始风速

        # RTD温度数据
        self.rtd_temps = [0.0] * 4  # 第5-8路温度(℃)

        # 数据锁
        self.lock = threading.Lock()

        # 时间戳
        self.last_update_time = time.time()

        # 用于绘图的历史数据（保存最近300个数据点）
        self.plot_data_length = 300
        self.time_history = deque(maxlen=self.plot_data_length)
        self.wind_raw_history = [deque(maxlen=self.plot_data_length) for _ in range(4)]
        self.wind_filtered_history = [deque(maxlen=self.plot_data_length) for _ in range(4)]
        self.wind_corrected_history = [deque(maxlen=self.plot_data_length) for _ in range(4)]


# --------------------------
# 模拟量传感器读取类
# --------------------------
class AnalogSensorReader:
    def __init__(self, shared_data: SharedData):
        self.shared_data = shared_data

        # 设备参数
        self.DEVICE_IP = "192.168.0.101"    # 设备IP
        self.DEVICE_PORT = 8234           # 设备Modbus端口
        self.SLAVE_ADDR = 1               # 设备从站地址
        self.FUNC_CODE = 0x04             # 功能码
        self.START_REG = 0                # 起始寄存器地址
        self.REG_COUNT = 12               # 读取寄存器数量
        self.READ_INTERVAL = 0.1          # 读取间隔（秒）
        self.TIMEOUT = 1                  # 单次读写超时时间
        self.BUFFER_SIZE = 1024
        self.RECONNECT_ATTEMPT = 1        # 连接断开后的重连次数

        # 全局变量
        self.last_registers: List[int] = []
        self.read_count = 0
        self.success_count = 0
        self.fail_count = 0
        self.sock: Optional[socket.socket] = None
        self.running = False

        # 卡尔曼滤波器初始化 - 仅对风速进行滤波
        self.wind_filters = [create_wind_speed_filter() for _ in range(4)]  # 4个风速滤波器

        # 颜色编码
        self.RED = "\033[91m"
        self.GREEN = "\033[92m"
        self.YELLOW = "\033[93m"
        self.RESET = "\033[0m"

    def connect_device(self) -> bool:
        """建立设备连接"""
        try:
            # 关闭原有连接（如果存在）
            if self.sock:
                try:
                    self.sock.close()
                except:
                    pass

            # 创建新连接
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(self.TIMEOUT)
            self.sock.connect((self.DEVICE_IP, self.DEVICE_PORT))
            return True
        except ConnectionRefusedError:
            print(f"[模拟量传感器] {self.RED}❌ 连接失败: 设备拒绝连接{self.RESET}")
        except TimeoutError:
            print(f"[模拟量传感器] {self.RED}❌ 连接失败: 连接超时{self.RESET}")
        except OSError as e:
            print(f"[模拟量传感器] {self.RED}❌ 连接失败: 网络错误 - {str(e)}{self.RESET}")
        except Exception as e:
            print(f"[模拟量传感器] {self.RED}❌ 连接失败: 未知错误 - {str(e)}{self.RESET}")
        return False

    def read_sensors(self):
        """读取一次传感器数据"""
        self.read_count += 1

        try:
            if not self.running:
                return

            if not self.sock:
                if not self.connect_device():
                    self.fail_count += 1
                    return

            # 构建请求
            request = build_rtu_request(
                slave_addr=self.SLAVE_ADDR,
                start_reg=self.START_REG,
                reg_count=self.REG_COUNT,
                func_code=self.FUNC_CODE
            )

            # 发送请求
            self.sock.sendall(request)

            # 接收响应
            response_bytes = b""
            request_start_time = time.time()

            while True:
                if not self.running:
                    return

                chunk = self.sock.recv(self.BUFFER_SIZE)
                if chunk:
                    response_bytes += chunk
                    # 检查完整帧
                    if len(response_bytes) >= 5:
                        data_len = response_bytes[2]
                        full_frame_len = 1 + 1 + 1 + data_len + 2  # 地址+功能码+字节数+数据+CRC
                        if len(response_bytes) >= full_frame_len:
                            break

                # 超时判断
                if time.time() - request_start_time > self.TIMEOUT:
                    raise socket.timeout(f"接收超时（{self.TIMEOUT}秒）")
                time.sleep(0.01)

            # 解析响应
            parsed_data = parse_rtu_response(response_bytes)
            if "error" in parsed_data:
                self.fail_count += 1
                return None

            # 提取数据
            registers = parsed_data["registers"]
            min_required = 2  # 至少需要温度和压力
            if len(registers) < min_required:
                self.fail_count += 1
                return None

            # 数据转换
            # 第1路：温度传感器
            temperature = 0.0
            if len(registers) > 0:
                temp_raw = registers[0]
                temp_current = temp_raw / 249
                temperature = (temp_current - 4) * 7.5 - 40

            # 第2路：压力传感器
            pressure = 0.0
            if len(registers) > 1:
                pressure_raw = registers[1]
                pressure_current = pressure_raw / 249
                pressure = (pressure_current - 4) * 7.5

            # 第5-8路：风速传感器
            wind_speeds = []
            wind_speeds_raw = []
            for i in range(4, 8):
                if i < len(registers):
                    raw_value = registers[i]
                    current_value = raw_value / 249
                    wind_speed_raw = (current_value - 4) * 30 / 16
                    wind_speed = self.wind_filters[i-4].update(wind_speed_raw)
                    wind_speeds.append(wind_speed)
                    wind_speeds_raw.append(wind_speed_raw)
                else:
                    wind_speeds.append(0.0)
                    wind_speeds_raw.append(0.0)

            # 第11路：湿度传感器
            humidity = 0.0
            if len(registers) > 10:
                humidity_raw = registers[10]
                humidity_current = humidity_raw / 249
                humidity = (humidity_current - 4) * 100 / 16

            # 更新共享数据
            with self.shared_data.lock:
                self.shared_data.analog_temp = temperature
                self.shared_data.analog_pressure = pressure
                self.shared_data.analog_humidity = humidity
                self.shared_data.wind_speeds = wind_speeds
                self.shared_data.wind_speeds_raw = wind_speeds_raw
                self.shared_data.last_update_time = time.time()

            self.last_registers = registers.copy()
            self.success_count += 1

            return True

        except Exception as e:
            self.fail_count += 1
            return None

    def run(self):
        """持续读取传感器数据"""
        self.running = True
        print(f"{self.GREEN}✅ 模拟量传感器连接成功！{self.RESET}")

        while self.running:
            self.read_sensors()
            # 使用可中断的睡眠
            for _ in range(int(self.READ_INTERVAL * 10)):
                if not self.running:
                    break
                time.sleep(0.1)

        # 关闭连接
        if self.sock:
            try:
                self.sock.close()
                print(f"[模拟量传感器] {self.GREEN}🔌 连接已关闭{self.RESET}")
            except:
                pass


# --------------------------
# RTD温度传感器读取类
# --------------------------
class RTDTemperatureReader:
    def __init__(self, shared_data: SharedData):
        self.shared_data = shared_data

        # 设备参数
        self.DEVICE_IP = "192.168.1.101"    # RTC模块IP地址
        self.DEVICE_PORT = 8234           # 设备Modbus端口
        self.SLAVE_ADDR = 1               # 设备从站地址
        self.FUNC_CODE = 0x04             # 功能码
        self.START_REG = 0                # 起始寄存器地址
        self.REG_COUNT = 12               # 读取寄存器数量
        self.READ_INTERVAL = 1            # 读取间隔（秒）
        self.TIMEOUT = 1                  # 单次读写超时时间
        self.BUFFER_SIZE = 1024
        self.RECONNECT_ATTEMPT = 1

        # 仅读取第5-8路
        self.DISPLAY_START_CH = 5
        self.DISPLAY_END_CH = 8

        # 全局变量
        self.last_temperatures: List[Optional[float]] = [None] * 12
        self.last_registers: List[int] = []
        self.read_count = 0
        self.success_count = 0
        self.fail_count = 0
        self.sock: Optional[socket.socket] = None
        self.running = False

        # 颜色编码
        self.RED = "\033[91m"
        self.GREEN = "\033[92m"
        self.YELLOW = "\033[93m"
        self.RESET = "\033[0m"

    def connect_device(self) -> bool:
        """建立设备连接"""
        try:
            if self.sock:
                try:
                    self.sock.close()
                except:
                    pass

            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(self.TIMEOUT)
            self.sock.connect((self.DEVICE_IP, self.DEVICE_PORT))
            return True
        except Exception as e:
            print(f"[RTD温度] {self.RED}❌ 连接失败: {str(e)}{self.RESET}")
            return False

    def read_sensors(self):
        """读取一次温度传感器数据"""
        self.read_count += 1

        try:
            if not self.running:
                return

            if not self.sock:
                if not self.connect_device():
                    self.fail_count += 1
                    return None

            # 发送请求
            request = build_rtu_request(
                slave_addr=self.SLAVE_ADDR,
                start_reg=self.START_REG,
                reg_count=self.REG_COUNT,
                func_code=self.FUNC_CODE
            )
            self.sock.sendall(request)

            # 接收响应
            response_bytes = b""
            request_start_time = time.time()

            while True:
                if not self.running:
                    return

                chunk = self.sock.recv(self.BUFFER_SIZE)
                if chunk:
                    response_bytes += chunk
                    if len(response_bytes) >= 5:
                        data_len = response_bytes[2]
                        full_frame_len = 1 + 1 + 1 + data_len + 2
                        if len(response_bytes) >= full_frame_len:
                            break

                if time.time() - request_start_time > self.TIMEOUT:
                    raise socket.timeout(f"接收超时（{self.TIMEOUT}秒）")
                time.sleep(0.01)

            # 解析响应
            parsed_data = parse_rtu_response(response_bytes)
            if "error" in parsed_data:
                self.fail_count += 1
                return None

            # 提取数据
            registers = parsed_data["registers"]
            if len(registers) < self.REG_COUNT:
                self.fail_count += 1
                return None

            # 数据转换（RTC温度专用公式）
            temperatures = []
            for i in range(12):
                temp_raw = registers[i]
                temperature = temp_raw / 10  # RTC温度转换公式
                temperatures.append(temperature)

            # 更新共享数据中的RTD温度（第5-8路）
            with self.shared_data.lock:
                for i in range(4):
                    self.shared_data.rtd_temps[i] = temperatures[4 + i]  # 索引4-7对应第5-8路
                self.shared_data.last_update_time = time.time()

            self.last_temperatures = temperatures.copy()
            self.last_registers = registers.copy()
            self.success_count += 1

            return temperatures[4:8]

        except Exception as e:
            self.fail_count += 1
            return None

    def run(self):
        """持续读取温度数据"""
        self.running = True
        print(f"{self.GREEN}✅ RTD温度传感器连接成功！{self.RESET}")

        while self.running:
            self.read_sensors()
            # 使用可中断的睡眠
            for _ in range(int(self.READ_INTERVAL * 10)):
                if not self.running:
                    break
                time.sleep(0.1)

        # 关闭连接
        if self.sock:
            try:
                self.sock.close()
                print(f"[RTD温度] {self.GREEN}🔌 连接已关闭{self.RESET}")
            except:
                pass


# --------------------------
# 数据计算和显示类
# --------------------------
class DataProcessor:
    def __init__(self, shared_data: SharedData):
        self.shared_data = shared_data
        self.running = False

        # 上一次的显示数据（用于高亮变化）
        self.last_display_data = [None] * 4  # 存储4路的上次显示数据

        # 标定环境参数
        self.CALIBRATION_TEMP = 23.1      # 标定温度(℃)
        self.CALIBRATION_RH = 0.65        # 标定相对湿度(65%)
        self.CALIBRATION_PRESSURE = 101.325  # 标定大气压力(kPa)

        # 计算标定空气密度
        self.calibration_density = self.calculate_air_density(
            self.CALIBRATION_TEMP,
            self.CALIBRATION_PRESSURE,
            self.CALIBRATION_RH * 100
        )

        # 颜色编码
        self.RED = "\033[91m"
        self.GREEN = "\033[92m"
        self.YELLOW = "\033[93m"
        self.RESET = "\033[0m"

    def calculate_air_density(self, temperature: float, pressure: float, humidity: float) -> float:
        """计算给定温度、压力和湿度下的空气密度"""
        try:
            # 创建AIR对象
            air = AIR(dP=pressure, unit='c', dTdb=temperature, dRh=humidity/100)
            air.updateData()
            prop = air.getProp(unit='c')
            return prop['Density(kg/m3)']
        except:
            return 0.0

    def run(self):
        """持续计算和显示数据"""
        self.running = True
        display_count = 0

        print(f"\n{self.GREEN}📊 开始显示传感器数据...{self.RESET}")
        print("="*90)
        print(f"标定环境参数: 温度 {self.CALIBRATION_TEMP}℃ | 湿度 {self.CALIBRATION_RH*100:.0f}% | 压力 {self.CALIBRATION_PRESSURE:.3f}kPa")
        print(f"标定空气密度: {self.calibration_density:.3f} kg/m³")
        print("="*90)

        while self.running:
            display_count += 1
            current_time = time.strftime("%H:%M:%S", time.localtime())

            # 获取共享数据的副本
            with self.shared_data.lock:
                rtd_temps = self.shared_data.rtd_temps.copy()
                wind_speeds = self.shared_data.wind_speeds.copy()
                wind_speeds_raw = self.shared_data.wind_speeds_raw.copy()
                pressure = self.shared_data.analog_pressure
                humidity = self.shared_data.analog_humidity
                analog_temp = self.shared_data.analog_temp  # 模拟量第1路温度
                update_time = self.shared_data.last_update_time

            # 计算模拟量温度对应的空气密度
            analog_density = self.calculate_air_density(analog_temp, pressure, humidity)

            # 构建模拟量温度显示行
            analog_temp_str = f"{analog_temp:5.1f}℃"
            analog_density_str = f"{analog_density:6.3f}kg/m³"

            # 高亮模拟量温度变化
            if hasattr(self, 'last_analog_data'):
                last_temp, last_density = self.last_analog_data
                if abs(analog_temp - last_temp) > 0.1:
                    analog_temp_str = f"{self.RED}{analog_temp:5.1f}℃{self.RESET}"
                if abs(analog_density - last_density) > 0.001:
                    analog_density_str = f"{self.RED}{analog_density:6.3f}kg/m³{self.RESET}"

            # 存储模拟量数据
            self.last_analog_data = (analog_temp, analog_density)

            # 打印标题
            header = f"[{current_time}] ✅ 第{display_count:03d}次 | 压力:{pressure:5.1f}kPa | 湿度:{humidity:5.1f}%"
            print(header)

            # 打印模拟量温度和对应的空气密度
            analog_line = f"  模拟量温度: {analog_temp_str} | 空气密度: {analog_density_str}"
            print(analog_line)
            print("-" * 90)  # 分隔线

            # 计算每个RTD位置对应的空气密度和修正风速
            densities = []
            display_lines = []
            corrected_wind_speeds = []  # 保存修正后的风速

            for i in range(4):  # 4个传感器
                # 计算该RTD温度下的空气密度
                density = self.calculate_air_density(rtd_temps[i], pressure, humidity)
                densities.append(density)

                # 计算修正系数K和修正后的风速
                # K = √(标定空气密度 / 实时空气密度)
                K = (self.calibration_density / density) ** 0.5 if density > 0 else 1.0
                corrected_wind_speed = wind_speeds[i] * K
                corrected_wind_speeds.append(corrected_wind_speed)

                # 构建显示字符串
                rtd_temp_str = f"{rtd_temps[i]:5.1f}℃"
                wind_str = f"{wind_speeds_raw[i]:5.1f}→{wind_speeds[i]:5.1f}m/s"
                density_str = f"{density:6.3f}kg/m³"
                k_str = f"{K:5.3f}"
                corrected_wind_str = f"{corrected_wind_speed:5.1f}m/s"

                # 高亮变化数据
                if self.last_display_data[i]:
                    last_temp, last_wind, last_density, last_k, last_corrected = self.last_display_data[i]

                    # 温度变化
                    if abs(rtd_temps[i] - last_temp) > 0.1:
                        rtd_temp_str = f"{self.RED}{rtd_temps[i]:5.1f}℃{self.RESET}"

                    # 风速变化
                    if abs(wind_speeds[i] - last_wind) > 0.1:
                        wind_str = f"{self.RED}{wind_speeds_raw[i]:5.1f}→{wind_speeds[i]:5.1f}m/s{self.RESET}"

                    # 密度变化
                    if abs(density - last_density) > 0.001:
                        density_str = f"{self.RED}{density:6.3f}kg/m³{self.RESET}"

                    # 修正系数变化
                    if abs(K - last_k) > 0.01:
                        k_str = f"{self.RED}{K:5.3f}{self.RESET}"

                    # 修正后风速变化
                    if abs(corrected_wind_speed - last_corrected) > 0.1:
                        corrected_wind_str = f"{self.RED}{corrected_wind_speed:5.1f}m/s{self.RESET}"

                # 存储当前显示数据
                self.last_display_data[i] = (rtd_temps[i], wind_speeds[i], density, K, corrected_wind_speed)

                # 构建显示行 - 显示修正前后的对比
                line = f"  RTD{i+5:02d}: {rtd_temp_str} | 风速{i+1}: {wind_str} | K值:{k_str} | 修正后:{corrected_wind_str} | 空气密度: {density_str}"
                display_lines.append(line)

            # 打印4个传感器的数据
            for line in display_lines:
                print(line)

            print()  # 空行分隔

            # 更新绘图数据
            with self.shared_data.lock:
                current_time = time.time()
                self.shared_data.time_history.append(current_time)
                for i in range(4):
                    self.shared_data.wind_raw_history[i].append(wind_speeds_raw[i])
                    self.shared_data.wind_filtered_history[i].append(wind_speeds[i])
                    self.shared_data.wind_corrected_history[i].append(corrected_wind_speeds[i])

            # 等待下一次显示
            for _ in range(10):  # 1秒间隔，每0.1秒检查一次
                if not self.running:
                    break
                time.sleep(0.1)


# --------------------------
# 实时绘图类
# --------------------------
class WindSpeedPlotter:
    def __init__(self, shared_data: SharedData):
        self.shared_data = shared_data
        self.running = False

        # 创建图形和子图
        try:
            plt.style.use('seaborn-v0_8-darkgrid')
        except:
            plt.style.use('default')
        self.fig, self.axes = plt.subplots(2, 2, figsize=(15, 10))
        self.fig.suptitle('风速实时监测 - 原始值/滤波值/修正后', fontsize=16)

        # 扁平化axes数组以便于索引
        self.axes = self.axes.flatten()

        # 每个子图的标题
        self.titles = ['风速传感器 1 (RTD05)', '风速传感器 2 (RTD06)',
                       '风速传感器 3 (RTD07)', '风速传感器 4 (RTD08)']

        # 初始化每条线
        self.lines_raw = []
        self.lines_filtered = []
        self.lines_corrected = []

        for i, ax in enumerate(self.axes):
            ax.set_title(self.titles[i])
            ax.set_xlabel('时间 (秒)')
            ax.set_ylabel('风速 (m/s)')
            ax.grid(True, alpha=0.3)

            # 创建三条线
            line_raw, = ax.plot([], [], 'r-', label='原始值', alpha=0.7, linewidth=1)
            line_filtered, = ax.plot([], [], 'b-', label='滤波后', linewidth=2)
            line_corrected, = ax.plot([], [], 'g-', label='修正后', linewidth=2)

            self.lines_raw.append(line_raw)
            self.lines_filtered.append(line_filtered)
            self.lines_corrected.append(line_corrected)

            # 添加图例
            ax.legend(loc='upper right')

            # 设置y轴范围
            ax.set_ylim(-5, 20)

        # 调整子图间距
        plt.tight_layout()

    def update_plot(self, frame):
        """更新绘图数据"""
        # 获取最新数据
        with self.shared_data.lock:
            if len(self.shared_data.time_history) > 0:
                time_data = list(self.shared_data.time_history)

                for i in range(4):
                    # 获取风速数据
                    raw_data = list(self.shared_data.wind_raw_history[i])
                    filtered_data = list(self.shared_data.wind_filtered_history[i])
                    corrected_data = list(self.shared_data.wind_corrected_history[i])

                    # 计算相对时间（秒）
                    if time_data:
                        relative_time = [(t - time_data[0]) for t in time_data]
                    else:
                        relative_time = []

                    # 更新线条数据
                    self.lines_raw[i].set_data(relative_time, raw_data)
                    self.lines_filtered[i].set_data(relative_time, filtered_data)
                    self.lines_corrected[i].set_data(relative_time, corrected_data)

                    # 自动调整x轴范围
                    if relative_time:
                        self.axes[i].set_xlim(max(0, relative_time[-1] - 60), relative_time[-1] + 1)

                        # 自动调整y轴范围
                        all_data = raw_data + filtered_data + corrected_data
                        if all_data:
                            y_min = min(all_data) - 1
                            y_max = max(all_data) + 1
                            self.axes[i].set_ylim(y_min, y_max)

        return self.lines_raw + self.lines_filtered + self.lines_corrected

    def run(self):
        """运行绘图"""
        self.running = True

        # 创建动画
        self.ani = animation.FuncAnimation(
            self.fig, self.update_plot,
            interval=100,  # 每100ms更新一次
            blit=True,
            cache_frame_data=False
        )

        plt.show()


# --------------------------
# 主程序
# --------------------------
def main():
    print("="*90)
    print("🚀 启动 [RTD温度-风速-空气密度] 监测系统")
    print("="*90)
    print("📡 系统功能：")
    print("   • 读取4路RTD温度（第5-8路）")
    print("   • 读取4路风速传感器（第5-8路）")
    print("   • 读取压力和湿度")
    print("   • 为每个RTD温度计算对应的空气密度")
    print("="*90)

    # 创建共享数据对象
    shared_data = SharedData()

    # 创建读取器、处理器和绘图器
    analog_reader = AnalogSensorReader(shared_data)
    rtd_reader = RTDTemperatureReader(shared_data)
    processor = DataProcessor(shared_data)
    plotter = WindSpeedPlotter(shared_data)

    # 创建线程
    analog_thread = threading.Thread(target=analog_reader.run)
    rtd_thread = threading.Thread(target=rtd_reader.run)
    processor_thread = threading.Thread(target=processor.run)
    plotter_thread = threading.Thread(target=plotter.run)

    try:
        # 连接设备
        print("\n📞 正在连接设备...")

        print(f"\n📡 连接模拟量传感器 (192.168.0.101)...")
        if analog_reader.connect_device():
            print(f"{analog_reader.GREEN}✅ 模拟量传感器连接成功！{analog_reader.GREEN}")
        else:
            print(f"{analog_reader.RED}❌ 模拟量传感器连接失败！{analog_reader.RESET}")
            return

        print(f"\n📡 连接RTD温度传感器 (192.168.1.101)...")
        if rtd_reader.connect_device():
            print(f"{rtd_reader.GREEN}✅ RTD温度传感器连接成功！{rtd_reader.RESET}")
        else:
            print(f"{rtd_reader.RED}❌ RTD温度传感器连接失败！{rtd_reader.RESET}")
            return

        # 启动线程
        analog_thread.start()
        rtd_thread.start()
        time.sleep(0.5)  # 等待数据稳定
        processor_thread.start()
        time.sleep(1)  # 等待一些数据累积
        plotter_thread.start()

        # 等待线程结束
        analog_thread.join()
        rtd_thread.join()
        processor_thread.join()
        plotter_thread.join()

    except KeyboardInterrupt:
        print(f"\n{analog_reader.YELLOW}⚠️  用户中断，正在停止程序...{analog_reader.RESET}")

        # 停止运行
        analog_reader.running = False
        rtd_reader.running = False
        processor.running = False
        plotter.running = False

        # 关闭matplotlib窗口
        plt.close('all')

        # 等待线程结束
        analog_thread.join(timeout=2)
        rtd_thread.join(timeout=2)
        processor_thread.join(timeout=2)
        plotter_thread.join(timeout=2)

    # 最终统计报告
    print("\n" + "="*90)
    print("📋 传感器读取结束 - 统计报告")
    print("="*90)

    print("\n📊 模拟量传感器统计：")
    print(f"🔢 总读取次数: {analog_reader.read_count}")
    print(f"✅ 成功次数: {analog_reader.success_count}")
    print(f"❌ 失败次数: {analog_reader.fail_count}")
    success_rate = (analog_reader.success_count / analog_reader.read_count * 100) if analog_reader.read_count > 0 else 0.0
    print(f"📈 成功率: {success_rate:.1f}%")

    print("\n📊 RTD温度传感器统计：")
    print(f"🔢 总读取次数: {rtd_reader.read_count}")
    print(f"✅ 成功次数: {rtd_reader.success_count}")
    print(f"❌ 失败次数: {rtd_reader.fail_count}")
    success_rate = (rtd_reader.success_count / rtd_reader.read_count * 100) if rtd_reader.read_count > 0 else 0.0
    print(f"📈 成功率: {success_rate:.1f}%")

    print("\n" + "="*90)


if __name__ == "__main__":
    main()