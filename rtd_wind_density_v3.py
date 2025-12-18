"""
RTD Temperature-based Wind Speed Monitoring with Air Density Correction
Combines wind speed sensors with RTD temperature sensors for accurate air density correction
"""

import socket
import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
from kalman_filter import create_wind_speed_filter
from typing import List, Optional, Tuple
from Refrigerant import AIR

# Modbus RTU frame processing functions
def modbus_crc(data: List[int]) -> List[int]:
    """Calculate Modbus CRC checksum"""
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
    """Build Modbus RTU request frame"""
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

def parse_rtu_response(response_bytes: bytes) -> dict:
    """Parse Modbus RTU response frame"""
    response = list(response_bytes)
    if len(response) < 4:
        return {"error": "Response frame too short"}

    slave_addr = response[0]
    func_code = response[1]
    data = response[2:-2]
    received_crc = response[-2:]

    calculated_crc = modbus_crc(response[:-2])
    if received_crc != calculated_crc:
        return {"error": "CRC check failed"}

    if func_code in [0x03, 0x04]:
        if len(data) < 1:
            return {"error": f"Function code {func_code:02X} response data is empty"}
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
        return {"error": f"Unsupported function code: 0x{func_code:02X}"}

def calculate_air_density(temperature: float, pressure: float, humidity: float) -> float:
    """Calculate air density using psychrometric formulas"""
    try:
        air = AIR(dP=pressure, unit='c', dTdb=temperature, dRh=humidity)
        air.updateData()
        prop = air.getProp(unit='c')
        return prop['Density(kg/m3)']
    except:
        return 1.195  # Default air density at calibration conditions

class RTDWindDensityPlotter:
    def __init__(self):
        # Wind sensor device parameters (like plot_wind_speed.py)
        self.WIND_DEVICE_IP = "192.168.0.101"
        self.WIND_DEVICE_PORT = 8234
        self.WIND_SLAVE_ADDR = 1
        self.WIND_FUNC_CODE = 0x04
        self.WIND_START_REG = 0
        self.WIND_REG_COUNT = 8
        self.TIMEOUT = 5
        self.BUFFER_SIZE = 1024

        # RTD device parameters
        self.RTD_DEVICE_IP = "192.168.0.100"
        self.RTD_DEVICE_PORT = 502
        self.RTD_SLAVE_ADDR = 1
        self.RTD_FUNC_CODE = 0x04
        self.RTD_START_REG = 0
        self.RTD_REG_COUNT = 8

        # Calibration parameters
        self.CALIBRATION_TEMP = 23.1  # °C
        self.CALIBRATION_RH = 0.65     # 65%
        self.CALIBRATION_PRESSURE = 101.325  # kPa
        self.calibration_density = calculate_air_density(
            self.CALIBRATION_TEMP,
            self.CALIBRATION_PRESSURE,
            self.CALIBRATION_RH * 100
        )

        # Data storage (following plot_wind_speed.py pattern)
        self.max_points = 2000
        self.time_data = deque()

        # Wind speed data series
        self.wind_raw_data = [deque() for _ in range(4)]
        self.wind_filtered_data = [deque() for _ in range(4)]
        self.wind_corrected_data = [deque() for _ in range(4)]

        # RTD temperature data
        self.rtd_temp_data = [deque() for _ in range(4)]

        # Environmental data
        self.pressure_data = deque()
        self.humidity_data = deque()
        self.density_data = deque()

        # Kalman filters for wind speeds
        self.wind_filters = [create_wind_speed_filter() for _ in range(4)]

        # Connection objects
        self.wind_sock: Optional[socket.socket] = None
        self.rtd_sock: Optional[socket.socket] = None
        self.wind_connected = False
        self.rtd_connected = False

        # Statistics
        self.read_count = 0
        self.success_count = 0
        self.fail_count = 0
        self.start_time = time.time()

        # Setup matplotlib (following plot_wind_speed.py pattern)
        plt.style.use('default')
        self.fig, self.axes = plt.subplots(2, 2, figsize=(14, 10))
        self.fig.suptitle('RTD Wind Speed Monitoring with Air Density Correction\n'
                         f'Calibration: {self.CALIBRATION_TEMP}°C, {self.CALIBRATION_RH*100}%RH, {self.CALIBRATION_PRESSURE}kPa',
                         fontsize=14, fontweight='bold')

        plt.subplots_adjust(hspace=0.3, wspace=0.25)

        # Initialize plot elements
        self.lines_raw = []
        self.lines_filtered = []
        self.lines_corrected = []
        self.current_value_texts = []
        self.density_texts = []

        colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728']
        titles = ['风速传感器 1 (RTD05)', '风速传感器 2 (RTD06)',
                 '风速传感器 3 (RTD07)', '风速传感器 4 (RTD08)']

        for i, ax in enumerate(self.axes.flat):
            ax.set_title(titles[i], fontsize=12)
            ax.set_xlabel('时间 (秒)', fontsize=10)
            ax.set_ylabel('风速 (m/s)', fontsize=10)
            ax.grid(True, alpha=0.3)

            # Create line objects
            line_raw, = ax.plot([], [], color=colors[i], alpha=0.4, linewidth=1, label='原始值')
            line_filtered, = ax.plot([], [], color=colors[i], linewidth=2, label='滤波后')
            line_corrected, = ax.plot([], [], color='red', linewidth=2.5, label='密度修正后')

            self.lines_raw.append(line_raw)
            self.lines_filtered.append(line_filtered)
            self.lines_corrected.append(line_corrected)

            # Add legend
            ax.legend(loc='upper right')

            # Initialize text objects for current values
            text_obj = ax.text(0.02, 0.95, '', transform=ax.transAxes,
                             verticalalignment='top', fontsize=9,
                             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
            self.current_value_texts.append(text_obj)

            # Initialize density text
            density_obj = ax.text(0.98, 0.95, '', transform=ax.transAxes,
                                 verticalalignment='top', horizontalalignment='right', fontsize=9,
                                 bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.8))
            self.density_texts.append(density_obj)

            # Set initial y-axis range
            ax.set_ylim(-2, 20)

        # Connect to devices
        self.connect_devices()

    def connect_devices(self) -> bool:
        """Connect to both wind sensor and RTD devices"""
        success = True

        # Connect to wind sensor device
        try:
            if self.wind_sock:
                self.wind_sock.close()
            self.wind_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.wind_sock.settimeout(self.TIMEOUT)
            self.wind_sock.connect((self.WIND_DEVICE_IP, self.WIND_DEVICE_PORT))
            self.wind_connected = True
            print(f"成功连接到风速设备 {self.WIND_DEVICE_IP}:{self.WIND_DEVICE_PORT}")
        except Exception as e:
            print(f"风速设备连接失败: {str(e)}")
            self.wind_connected = False
            success = False

        # Connect to RTD device
        try:
            if self.rtd_sock:
                self.rtd_sock.close()
            self.rtd_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.rtd_sock.settimeout(self.TIMEOUT)
            self.rtd_sock.connect((self.RTD_DEVICE_IP, self.RTD_DEVICE_PORT))
            self.rtd_connected = True
            print(f"成功连接到RTD设备 {self.RTD_DEVICE_IP}:{self.RTD_DEVICE_PORT}")
        except Exception as e:
            print(f"RTD设备连接失败: {str(e)}")
            self.rtd_connected = False
            success = False

        return success

    def read_wind_data(self) -> Optional[Tuple]:
        """Read wind speed data from wind sensor device"""
        if not self.wind_connected or not self.wind_sock:
            return None

        try:
            # Build request
            request = build_rtu_request(
                slave_addr=self.WIND_SLAVE_ADDR,
                start_reg=self.WIND_START_REG,
                reg_count=self.WIND_REG_COUNT,
                func_code=self.WIND_FUNC_CODE
            )

            # Send request
            self.wind_sock.sendall(request)

            # Receive response
            response_bytes = b""
            request_start_time = time.time()

            while True:
                chunk = self.wind_sock.recv(self.BUFFER_SIZE)
                if chunk:
                    response_bytes += chunk
                    if len(response_bytes) >= 5:
                        data_len = response_bytes[2]
                        full_frame_len = 1 + 1 + 1 + data_len + 2
                        if len(response_bytes) >= full_frame_len:
                            break

                if time.time() - request_start_time > self.TIMEOUT:
                    raise socket.timeout("接收超时")
                time.sleep(0.01)

            # Parse response
            parsed_data = parse_rtu_response(response_bytes)
            if "error" in parsed_data:
                print(f"风速数据解析失败: {parsed_data['error']}")
                return None

            registers = parsed_data["registers"]
            if len(registers) < self.WIND_REG_COUNT:
                print(f"风速数据不足: 需要{self.WIND_REG_COUNT}个, 得到{len(registers)}个")
                return None

            # Extract wind speed data (registers 4-7)
            wind_speeds_raw = []
            for i in range(4, 8):
                raw_value = registers[i]
                current_value = raw_value / 249  # Convert to current (mA)
                wind_speed_raw = (current_value - 4) * 30 / 16  # Convert to wind speed (m/s)
                wind_speeds_raw.append(wind_speed_raw)

            # Extract pressure (register 1) and humidity (register 7)
            pressure_raw = registers[1]
            pressure_current = pressure_raw / 249
            pressure = (pressure_current - 4) * 7.5

            humidity_raw = registers[7] if len(registers) > 7 else 0
            humidity_current = humidity_raw / 249
            humidity = (humidity_current - 4) * 100 / 16

            return wind_speeds_raw, pressure, humidity

        except Exception as e:
            print(f"风速数据读取失败: {str(e)}")
            if "Connection" in str(e) or "reset" in str(e):
                self.wind_connected = False
                self.connect_devices()
            return None

    def read_rtd_data(self) -> Optional[List[float]]:
        """Read RTD temperature data"""
        if not self.rtd_connected or not self.rtd_sock:
            return None

        try:
            # Build request - read 8 registers starting from 0
            request = build_rtu_request(
                slave_addr=self.RTD_SLAVE_ADDR,
                start_reg=self.RTD_START_REG,
                reg_count=self.RTD_REG_COUNT,
                func_code=self.RTD_FUNC_CODE
            )

            # Send request
            self.rtd_sock.sendall(request)

            # Receive response
            response_bytes = b""
            request_start_time = time.time()

            while True:
                chunk = self.rtd_sock.recv(self.BUFFER_SIZE)
                if chunk:
                    response_bytes += chunk
                    if len(response_bytes) >= 5:
                        data_len = response_bytes[2]
                        full_frame_len = 1 + 1 + 1 + data_len + 2
                        if len(response_bytes) >= full_frame_len:
                            break

                if time.time() - request_start_time > self.TIMEOUT:
                    raise socket.timeout("RTD接收超时")
                time.sleep(0.01)

            # Parse response
            parsed_data = parse_rtu_response(response_bytes)
            if "error" in parsed_data:
                print(f"RTD数据解析失败: {parsed_data['error']}")
                return None

            registers = parsed_data["registers"]
            if len(registers) < self.RTD_REG_COUNT:
                print(f"RTD数据不足: 需要{self.RTD_REG_COUNT}个, 得到{len(registers)}个")
                return None

            # Extract RTD temperatures (registers 4-7 for channels 5-8)
            rtd_temps = []
            for i in range(4, 8):
                raw_value = registers[i]
                # Convert to temperature (assuming same conversion as sensor.py)
                temperature = raw_value / 10  # RTD temperature conversion
                rtd_temps.append(temperature)

            return rtd_temps

        except Exception as e:
            print(f"RTD数据读取失败: {str(e)}")
            if "Connection" in str(e) or "reset" in str(e):
                self.rtd_connected = False
                self.connect_devices()
            return None

    def update(self, frame):
        """Update data and plots"""
        self.frame_count = getattr(self, 'frame_count', 0) + 1
        current_time = time.time() - self.start_time

        # Read wind data
        wind_result = self.read_wind_data()

        # Read RTD data
        rtd_result = self.read_rtd_data()

        if wind_result and rtd_result:
            self.read_count += 1
            self.success_count += 1

            # Unpack data
            wind_speeds_raw = wind_result[0]
            pressure = wind_result[1] if len(wind_result) > 1 else self.CALIBRATION_PRESSURE
            humidity = wind_result[2] if len(wind_result) > 2 else self.CALIBRATION_RH * 100
            rtd_temps = rtd_result

            # Add timestamp
            self.time_data.append(current_time)

            # Update data storage
            for i in range(4):
                # Store raw wind speed
                self.wind_raw_data[i].append(wind_speeds_raw[i])

                # Apply Kalman filter
                wind_speed_filtered = self.wind_filters[i].update(wind_speeds_raw[i])
                self.wind_filtered_data[i].append(wind_speed_filtered)

                # Store RTD temperature
                self.rtd_temp_data[i].append(rtd_temps[i])

                # Calculate air density using RTD temperature
                density = calculate_air_density(rtd_temps[i], pressure, humidity)

                # Calculate K factor and corrected wind speed
                K = (self.calibration_density / density) ** 0.5 if density > 0 else 1.0
                wind_speed_corrected = wind_speed_filtered * K
                self.wind_corrected_data[i].append(wind_speed_corrected)

            # Store environmental data (using first sensor's values)
            self.pressure_data.append(pressure)
            self.humidity_data.append(humidity)
            avg_density = calculate_air_density(
                np.mean(rtd_temps), pressure, humidity
            )
            self.density_data.append(avg_density)

            # Update plots
            for i in range(4):
                # Update line data
                self.lines_raw[i].set_data(self.time_data, self.wind_raw_data[i])
                self.lines_filtered[i].set_data(self.time_data, self.wind_filtered_data[i])
                self.lines_corrected[i].set_data(self.time_data, self.wind_corrected_data[i])

                # Dynamic x-axis adjustment
                time_window = 60
                time_margin = 5

                if current_time > time_window * 0.8 and len(self.time_data) > 100:
                    x_min = current_time - time_window
                    x_max = current_time + time_margin
                    self.axes.flat[i].set_xlim(x_min, x_max)
                else:
                    self.axes.flat[i].set_xlim(0, max(time_window, current_time + time_margin))

                # Dynamic y-axis adjustment
                if len(self.wind_raw_data[i]) > 0:
                    recent_points = min(100, len(self.wind_raw_data[i]))
                    recent_raw = list(self.wind_raw_data[i])[-recent_points:]
                    recent_filtered = list(self.wind_filtered_data[i])[-recent_points:]
                    recent_corrected = list(self.wind_corrected_data[i])[-recent_points:]

                    if recent_raw and recent_filtered and recent_corrected:
                        all_data = recent_raw + recent_filtered + recent_corrected
                        y_min = min(all_data)
                        y_max = max(all_data)

                        # Add padding
                        y_range = y_max - y_min
                        if y_range < 2:
                            y_range = 2
                        y_min -= y_range * 0.1
                        y_max += y_range * 0.1

                        # Ensure minimum values
                        if y_min < 0: y_min = 0
                        if y_max > 30: y_max = 30

                        self.axes.flat[i].set_ylim(y_min, y_max)

                        # Update current value text
                        current_rtd = self.rtd_temp_data[i][-1]
                        current_density = calculate_air_density(current_rtd, pressure, humidity)
                        K = (self.calibration_density / current_density) ** 0.5 if current_density > 0 else 1.0

                        self.current_value_texts[i].set_text(
                            f'原始: {recent_raw[-1]:.2f} m/s\n'
                            f'滤波: {recent_filtered[-1]:.2f} m/s\n'
                            f'修正: {recent_corrected[-1]:.2f} m/s'
                        )

                        self.density_texts[i].set_text(
                            f'RTD: {current_rtd:.1f}°C\n'
                            f'ρ: {current_density:.3f} kg/m³\n'
                            f'K: {K:.3f}'
                        )

            # Update title with statistics
            success_rate = self.success_count / self.read_count * 100 if self.read_count > 0 else 0
            if len(self.density_data) > 0:
                current_density = self.density_data[-1]
                current_pressure = self.pressure_data[-1]
                current_humidity = self.humidity_data[-1]

                self.fig.suptitle(
                    f'RTD风速监测系统 | 成功率: {success_rate:.1f}% | '
                    f'读取次数: {self.read_count} | 运行时间: {current_time:.0f}s\n'
                    f'当前环境: 密度={current_density:.3f} kg/m³, 压力={current_pressure:.1f} kPa, 湿度={current_humidity:.1f}%RH',
                    fontsize=12, fontweight='bold'
                )

            # Print status
            if self.frame_count % 10 == 0:
                print(f"\r时间: {current_time:6.1f}s | "
                      f"风速修正后: {self.wind_corrected_data[0][-1]:5.2f} | "
                      f"{self.wind_corrected_data[1][-1]:5.2f} | "
                      f"{self.wind_corrected_data[2][-1]:5.2f} | "
                      f"{self.wind_corrected_data[3][-1]:5.2f} m/s", end='')
        else:
            self.read_count += 1
            self.fail_count += 1

        return self.lines_raw + self.lines_filtered + self.lines_corrected

    def start(self):
        """Start real-time plotting"""
        print("\n" + "="*60)
        print("RTD风速监测系统 - 带空气密度修正")
        print("="*60)
        print(f"风速设备: {self.WIND_DEVICE_IP}:{self.WIND_DEVICE_PORT}")
        print(f"RTD设备: {self.RTD_DEVICE_IP}:{self.RTD_DEVICE_PORT}")
        print(f"标定条件: {self.CALIBRATION_TEMP}°C, {self.CALIBRATION_RH*100}%RH, {self.CALIBRATION_PRESSURE}kPa")
        print(f"标定密度: {self.calibration_density:.3f} kg/m³")
        print("显示内容: 原始值、卡尔曼滤波值、基于RTD温度的密度修正值")
        print("按Ctrl+C停止")
        print("="*60)

        # Disable interactive mode
        plt.ioff()

        # Create animation
        self.ani = FuncAnimation(
            self.fig, self.update, interval=100,
            blit=False, cache_frame_data=False, repeat=True
        )

        try:
            plt.show()
        except KeyboardInterrupt:
            print("\n\n用户中断，停止程序...")
        finally:
            # Close connections
            if self.wind_sock:
                self.wind_sock.close()
            if self.rtd_sock:
                self.rtd_sock.close()

            # Print statistics
            total_runtime = time.time() - self.start_time
            success_rate = self.success_count / self.read_count * 100 if self.read_count > 0 else 0

            print("\n" + "="*60)
            print("统计报告")
            print("="*60)
            print(f"总运行时间: {total_runtime:.1f} 秒")
            print(f"总读取次数: {self.read_count}")
            print(f"成功次数: {self.success_count}")
            print(f"失败次数: {self.fail_count}")
            print(f"成功率: {success_rate:.1f}%")
            print(f"数据点存储: {len(self.time_data)}")
            print("="*60)

if __name__ == "__main__":
    plotter = RTDWindDensityPlotter()
    plotter.start()