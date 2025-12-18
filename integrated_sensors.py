import socket
import time
from typing import List, Dict, Optional
from Refrigerant import AIR
from kalman_filter import create_wind_speed_filter
import threading


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
# 模拟量传感器读取类
# --------------------------
class AnalogSensorReader:
    def __init__(self):
        # 设备参数
        self.DEVICE_IP = "192.168.0.101"    # 设备IP
        self.DEVICE_PORT = 8234           # 设备Modbus端口
        self.SLAVE_ADDR = 1               # 设备从站地址
        self.FUNC_CODE = 0x04             # 功能码
        self.START_REG = 0                # 起始寄存器地址
        self.REG_COUNT = 12               # 读取寄存器数量（尝试读取12个）
        self.READ_INTERVAL = 0.1          # 读取间隔（秒）
        self.TIMEOUT = 1                  # 单次读写超时时间（缩短为1秒以便快速响应停止）
        self.BUFFER_SIZE = 1024
        self.RECONNECT_ATTEMPT = 1        # 连接断开后的重连次数

        # 全局变量
        self.last_registers: List[int] = []  # 存储上一次读取的所有寄存器值
        self.last_air_density: float = 0.0  # 存储上一次的空气密度
        self.last_humidity: float = 0.0     # 存储上一次的湿度
        self.read_count = 0               # 总读取次数
        self.success_count = 0            # 成功次数
        self.fail_count = 0               # 失败次数
        self.sock: Optional[socket.socket] = None  # 连接对象
        self.running = False              # 运行状态
        self.start_time = time.time()     # 记录开始时间

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
            print(f"[模拟量传感器] {self.RED}❌ 连接失败: 设备拒绝连接（IP/端口错误或设备离线）{self.RESET}")
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
        current_time = time.strftime("%H:%M:%S", time.localtime())
        read_start_time = time.time()

        try:
            # 检查是否需要停止
            if not self.running:
                return None

            if not self.sock:
                if not self.connect_device():
                    self.fail_count += 1
                    return None

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
                # 检查是否需要停止
                if not self.running:
                    return None

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
                print(f"[模拟量传感器] [{current_time}] ❌ 第{self.read_count:03d}次: 解析失败 - {parsed_data['error']}")
                self.fail_count += 1
                return None

            # 提取数据
            registers = parsed_data["registers"]
            # 只需要确保至少有足够的基本数据（温度和压力）
            min_required = 2  # 至少需要温度和压力
            if len(registers) < min_required:
                print(f"[模拟量传感器] [{current_time}] ❌ 第{self.read_count:03d}次: 数据不足（实际{len(registers)}个，至少需要{min_required}个）")
                self.fail_count += 1
                return None

            # 数据转换
            # 第1路：温度传感器
            temperature = 0.0  # 默认值
            temp_raw = 0  # 默认值
            if len(registers) > 0:
                temp_raw = registers[0]
                temp_current = temp_raw / 249  # 转换为电流值(mA)
                temperature = (temp_current - 4) * 7.5 - 40

            # 第2路：压力传感器
            pressure = 0.0  # 默认值
            pressure_raw = 0  # 默认值
            if len(registers) > 1:
                pressure_raw = registers[1]
                pressure_current = pressure_raw / 249  # 转换为电流值(mA)
                pressure = (pressure_current - 4) * 7.5

            # 第5-8路：风速传感器（应用卡尔曼滤波）
            wind_speeds = []
            wind_speeds_raw = []
            for i in range(4, 8):
                if i < len(registers):  # 检查索引是否有效
                    raw_value = registers[i]
                    current_value = raw_value / 249
                    wind_speed_raw = (current_value - 4) * 30 / 16
                    wind_speed = self.wind_filters[i-4].update(wind_speed_raw)
                    wind_speeds.append(wind_speed)
                    wind_speeds_raw.append(wind_speed_raw)
                else:
                    wind_speeds.append(0.0)  # 默认值
                    wind_speeds_raw.append(0.0)

            # 第11路：湿度传感器（索引10）
            humidity = 0.0  # 默认值
            humidity_raw = 0  # 默认值
            if len(registers) > 10:  # 检查是否有第11个寄存器（索引10）
                humidity_raw = registers[10]
                humidity_current = humidity_raw / 249  # 转换为电流值(mA)
                humidity = (humidity_current - 4) * 100 / 16  # 湿度计算公式

            # 使用AIR类计算空气密度（使用实际湿度）
            try:
                air = AIR(dP=pressure, unit='c', dTdb=temperature, dRh=humidity/100)  # 使用实际湿度
                air.updateData()
                prop = air.getProp(unit='c')
                air_density = prop['Density(kg/m3)']
            except Exception as e:
                print(f"[模拟量传感器] [{current_time}] ⚠️ 空气密度计算失败: {str(e)}")
                air_density = 0.0

            read_duration = (time.time() - read_start_time) * 1000  # 毫秒

            # 高亮变化数据
            temp_str = f"{temperature:5.1f}℃"
            temp_raw_str = f"{temp_raw:4d}"
            pressure_str = f"{pressure:5.1f}kPa"
            pressure_raw_str = f"{pressure_raw:4d}"
            humidity_str = f"{humidity:5.1f}%"
            humidity_raw_str = f"{humidity_raw:4d}"
            density_str = f"{air_density:6.3f}kg/m³"

            wind_strs = []
            wind_raw_strs = []

            # 确保 wind_strs 总是有4个元素
            for i in range(4):
                if i < len(wind_speeds):
                    wind_strs.append(f"{wind_speeds_raw[i]:5.1f}→{wind_speeds[i]:5.1f}m/s")
                    if 4+i < len(registers):
                        wind_raw_strs.append(f"{registers[4+i]:4d}")
                    else:
                        wind_raw_strs.append(f"    ")
                else:
                    wind_strs.append(f"    0.0→    0.0m/s")
                    wind_raw_strs.append(f"    ")

            if self.last_registers and len(self.last_registers) > 0:
                # 检查温度变化
                if len(self.last_registers) > 0:
                    last_temp_current = self.last_registers[0] / 249
                    last_temp = (last_temp_current - 4) * 7.5 - 40
                    if abs(temperature - last_temp) > 0.1:
                        temp_str = f"{self.RED}{temperature:5.1f}℃{self.RESET}"
                        temp_raw_str = f"{self.RED}{temp_raw:4d}{self.RESET}"

                # 检查压力变化
                if len(self.last_registers) > 1:
                    last_pressure_current = self.last_registers[1] / 249
                    last_pressure = (last_pressure_current - 4) * 7.5
                    if abs(pressure - last_pressure) > 0.1:
                        pressure_str = f"{self.RED}{pressure:5.1f}kPa{self.RESET}"
                        pressure_raw_str = f"{self.RED}{pressure_raw:4d}{self.RESET}"

                # 检查湿度变化
                if len(self.last_registers) > 10 and len(registers) > 10:
                    last_humidity_current = self.last_registers[10] / 249
                    last_humidity = (last_humidity_current - 4) * 100 / 16
                    if abs(humidity - last_humidity) > 1:
                        humidity_str = f"{self.RED}{humidity:5.1f}%{self.RESET}"
                        humidity_raw_str = f"{self.RED}{humidity_raw:4d}{self.RESET}"

                # 检查空气密度变化
                if abs(air_density - self.last_air_density) > 0.01:
                    density_str = f"{self.RED}{air_density:5.2f}kg/m³{self.RESET}"

                # 检查风速变化
                for i, wind in enumerate(wind_speeds):
                    if len(self.last_registers) > (4+i) and len(registers) > (4+i):
                        last_current = self.last_registers[4+i] / 249
                        last_wind = (last_current - 4) * 30 / 16
                        if abs(wind - last_wind) > 0.1:
                            # 更新 wind_strs 中对应的元素
                            if i < len(wind_strs):
                                wind_strs[i] = f"{self.RED}{wind_speeds_raw[i]:5.1f}→{wind:5.1f}m/s{self.RESET}"
                            if i < len(wind_raw_strs):
                                wind_raw_strs[i] = f"{self.RED}{registers[4+i]:4d}{self.RESET}"

            # 打印结果
            output_line = f"[{current_time}] [模拟量] ✅ 第{self.read_count:03d}次 | 耗时:{read_duration:4.0f}ms | "
            output_line += f"温度:{temp_raw_str}→{temp_str} | "
            output_line += f"压力:{pressure_raw_str}→{pressure_str} | "
            output_line += f"湿度:{humidity_raw_str}→{humidity_str} | "
            output_line += f"空气密度:{density_str} | "
            output_line += f"风速:{wind_strs[0]} | {wind_strs[1]} | {wind_strs[2]} | {wind_strs[3]}"

            print(output_line)

            # 更新记录
            self.last_registers = registers.copy()
            self.last_air_density = air_density
            self.last_humidity = humidity
            self.success_count += 1

            return {
                'temperature': temperature,
                'pressure': pressure,
                'humidity': humidity,
                'air_density': air_density,
                'wind_speeds': wind_speeds,
                'timestamp': current_time
            }

        except socket.timeout as e:
            print(f"[模拟量传感器] [{current_time}] ⏰ 第{self.read_count:03d}次: 读取超时 - {str(e)}")
            self.fail_count += 1
        except ConnectionResetError:
            print(f"[模拟量传感器] [{current_time}] {self.RED}🚫 第{self.read_count:03d}次: 连接被设备重置{self.RESET}")
            # 尝试重连
            for attempt in range(self.RECONNECT_ATTEMPT):
                print(f"[模拟量传感器] [{current_time}] 🔄 正在重连（{attempt+1}/{self.RECONNECT_ATTEMPT}）...")
                if self.connect_device():
                    break
                time.sleep(2)
            self.fail_count += 1
        except OSError as e:
            print(f"[模拟量传感器] [{current_time}] {self.RED}❌ 第{self.read_count:03d}次: 网络错误 - {str(e)}{self.RESET}")
            self.fail_count += 1
        except Exception as e:
            # 捕获所有其他异常，避免程序崩溃
            import traceback
            print(f"[模拟量传感器] [{current_time}] {self.RED}❌ 第{self.read_count:03d}次: 未知异常 - {str(e)}（{type(e).__name__}）{self.RESET}")
            print(f"[模拟量传感器] 详细错误信息:")
            traceback.print_exc()
            if 'registers' in locals():
                print(f"[模拟量传感器] 寄存器数组长度: {len(registers)}")
                print(f"[模拟量传感器] 寄存器内容: {registers}")
            self.fail_count += 1

        return None

    def run(self):
        """持续读取传感器数据"""
        self.running = True
        print(f"\n{self.GREEN}✅ 模拟量传感器连接成功！{self.RESET}")

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
    def __init__(self):
        # 设备参数（务必与设备手册一致！）
        self.DEVICE_IP = "192.168.1.101"    # RTC模块IP地址
        self.DEVICE_PORT = 8234           # 设备Modbus端口
        self.SLAVE_ADDR = 1               # 设备从站地址
        self.FUNC_CODE = 0x04             # 功能码（0x03=保持寄存器，0x04=输入寄存器）
        self.START_REG = 0                # 12路温度起始寄存器地址
        self.REG_COUNT = 12               # 读取12路温度寄存器数量
        self.READ_INTERVAL = 1            # 读取间隔（秒）
        self.TIMEOUT = 1                  # 单次读写超时时间（缩短为1秒以便快速响应停止）
        self.BUFFER_SIZE = 1024
        self.RECONNECT_ATTEMPT = 1        # 连接断开后的重连次数

        # 修改：仅输出第5-8路温度
        self.DISPLAY_START_CH = 5         # 起始显示通道（第5路）
        self.DISPLAY_END_CH = 8           # 结束显示通道（第8路）

        # 全局变量（12路RTC专用）
        self.last_temperatures: List[Optional[float]] = [None] * 12  # 保存12路温度值
        self.last_registers: List[int] = []
        self.read_count = 0               # 总读取次数
        self.success_count = 0            # 成功次数
        self.fail_count = 0               # 失败次数
        self.sock: Optional[socket.socket] = None  # 连接对象
        self.running = False              # 运行状态
        self.start_time = time.time()     # 记录开始时间

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
            print(f"[RTD温度] {self.RED}❌ 连接失败: 设备拒绝连接（IP/端口错误或设备离线）{self.RESET}")
        except TimeoutError:
            print(f"[RTD温度] {self.RED}❌ 连接失败: 连接超时{self.RESET}")
        except OSError as e:
            print(f"[RTD温度] {self.RED}❌ 连接失败: 网络错误 - {str(e)}{self.RESET}")
        except Exception as e:
            print(f"[RTD温度] {self.RED}❌ 连接失败: 未知错误 - {str(e)}{self.RESET}")
        return False

    def read_sensors(self):
        """读取一次温度传感器数据"""
        self.read_count += 1
        current_time = time.strftime("%H:%M:%S", time.localtime())
        read_start_time = time.time()

        try:
            # 检查是否需要停止
            if not self.running:
                return None

            if not self.sock:
                if not self.connect_device():
                    self.fail_count += 1
                    return None

            # 发送请求（12路RTC专用寄存器配置）
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
                # 检查是否需要停止
                if not self.running:
                    return None

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
                print(f"[RTD温度] [{current_time}] ❌ 第{self.read_count:03d}次: 解析失败 - {parsed_data['error']}")
                self.fail_count += 1
                return None

            # 提取数据（12路RTC读取12个寄存器）
            registers = parsed_data["registers"]
            if len(registers) < self.REG_COUNT:
                print(f"[RTD温度] [{current_time}] ❌ 第{self.read_count:03d}次: 数据不足（实际{len(registers)}个，期望{self.REG_COUNT}个）")
                self.fail_count += 1
                return None

            # 数据转换（12路RTC温度专用公式：原始值÷10 = 实际温度）
            temperatures = []
            temp_display_strings = []

            for i in range(12):
                temp_raw = registers[i]
                temperature = temp_raw / 10  # RTC温度转换公式
                temperatures.append(temperature)

                # 高亮变化数据
                temp_str = f"{temperature:5.1f}℃"
                temp_raw_str = f"{temp_raw:4d}"

                if self.last_temperatures[i] is not None and abs(temperature - self.last_temperatures[i]) > 0.1:
                    temp_str = f"{self.RED}{temp_str}{self.RESET}"
                    temp_raw_str = f"{self.RED}{temp_raw_str}{self.RESET}"

                temp_display_strings.append(f"CH{i+1:02d}:{temp_raw_str}→{temp_str}")

            read_duration = (time.time() - read_start_time) * 1000  # 毫秒

            # 打印结果（仅显示第5-8路）
            header = f"[{current_time}] [RTD温度] ✅ 第{self.read_count:03d}次 | 耗时:{read_duration:4.0f}ms | 第{self.DISPLAY_START_CH}-{self.DISPLAY_END_CH}路温度传感器数据:"
            print(header)

            # 仅显示第5-8路传感器
            for i in range(self.DISPLAY_START_CH-1, self.DISPLAY_END_CH):
                print(f"    {temp_display_strings[i]}")

            print()  # 空行分隔

            # 更新记录
            self.last_temperatures = temperatures.copy()
            self.last_registers = registers.copy()
            self.success_count += 1

            return temperatures[4:8]  # 返回第5-8路的温度

        except socket.timeout as e:
            print(f"[RTD温度] [{current_time}] ⏰ 第{self.read_count:03d}次: 读取超时 - {str(e)}")
            self.fail_count += 1
        except ConnectionResetError:
            print(f"[RTD温度] [{current_time}] {self.RED}🚫 第{self.read_count:03d}次: 连接被设备重置{self.RESET}")
            # 尝试重连
            for attempt in range(self.RECONNECT_ATTEMPT):
                print(f"[RTD温度] [{current_time}] 🔄 正在重连（{attempt+1}/{self.RECONNECT_ATTEMPT}）...")
                if self.connect_device():
                    break
                time.sleep(2)
            self.fail_count += 1
        except OSError as e:
            print(f"[RTD温度] [{current_time}] {self.RED}❌ 第{self.read_count:03d}次: 网络错误 - {str(e)}{self.RESET}")
            self.fail_count += 1
        except Exception as e:
            print(f"[RTD温度] [{current_time}] {self.RED}❌ 第{self.read_count:03d}次: 未知异常 - {str(e)}（{type(e).__name__}）{self.RESET}")
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
# 主程序
# --------------------------
def main():
    print("="*80)
    print("🚀 启动 [集成传感器系统] 模式")
    print("="*80)
    print("📡 系统包含两个传感器模块：")
    print("   1. 模拟量传感器 (192.168.0.101) - 温度、压力、湿度、风速")
    print("   2. RTD温度传感器 (192.168.1.101) - 第5-8路温度")
    print("="*80)

    # 创建传感器读取器
    analog_reader = AnalogSensorReader()
    rtd_reader = RTDTemperatureReader()

    # 创建线程
    analog_thread = threading.Thread(target=analog_reader.run)
    rtd_thread = threading.Thread(target=rtd_reader.run)

    try:
        # 连接设备
        print("\n📞 正在连接模拟量传感器 (192.168.0.101)...")
        if analog_reader.connect_device():
            print(f"{analog_reader.GREEN}✅ 模拟量传感器连接成功！{analog_reader.RESET}")
        else:
            print(f"{analog_reader.RED}❌ 模拟量传感器连接失败！{analog_reader.RESET}")
            return

        print("\n📞 正在连接RTD温度传感器 (192.168.1.101)...")
        if rtd_reader.connect_device():
            print(f"{rtd_reader.GREEN}✅ RTD温度传感器连接成功！{rtd_reader.RESET}")
        else:
            print(f"{rtd_reader.RED}❌ RTD温度传感器连接失败！{rtd_reader.RESET}")
            return

        print("\n" + "="*80)
        print("📊 传感器数据读取开始...")
        print("="*80)

        # 启动线程
        analog_thread.start()
        time.sleep(0.5)  # 稍微延迟一下，避免输出混乱
        rtd_thread.start()

        # 等待线程结束
        analog_thread.join()
        rtd_thread.join()

    except KeyboardInterrupt:
        print(f"\n{analog_reader.YELLOW}⚠️  用户中断，正在停止程序...{analog_reader.RESET}")

        # 停止运行
        analog_reader.running = False
        rtd_reader.running = False

        # 等待线程结束
        if analog_thread.is_alive():
            analog_thread.join(timeout=2)
        if rtd_thread.is_alive():
            rtd_thread.join(timeout=2)

    # 最终统计报告
    print("\n" + "="*80)
    print("📋 传感器读取结束 - 统计报告")
    print("="*80)

    # 模拟量传感器统计
    print("\n📊 模拟量传感器统计：")
    print(f"🕐 总运行时间: {time.time() - (analog_reader.start_time if hasattr(analog_reader, 'start_time') else 0):.1f} 秒")
    print(f"🔢 总读取次数: {analog_reader.read_count}")
    print(f"✅ 成功次数: {analog_reader.success_count}")
    print(f"❌ 失败次数: {analog_reader.fail_count}")
    success_rate = (analog_reader.success_count / analog_reader.read_count * 100) if analog_reader.read_count > 0 else 0.0
    print(f"📈 成功率: {success_rate:.1f}%")

    # RTD温度传感器统计
    print("\n📊 RTD温度传感器统计：")
    print(f"🕐 总运行时间: {time.time() - (rtd_reader.start_time if hasattr(rtd_reader, 'start_time') else 0):.1f} 秒")
    print(f"🔢 总读取次数: {rtd_reader.read_count}")
    print(f"✅ 成功次数: {rtd_reader.success_count}")
    print(f"❌ 失败次数: {rtd_reader.fail_count}")
    success_rate = (rtd_reader.success_count / rtd_reader.read_count * 100) if rtd_reader.read_count > 0 else 0.0
    print(f"📈 成功率: {success_rate:.1f}%")

    print("\n" + "="*80)


if __name__ == "__main__":
    main()