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
from pathlib import Path
from datetime import datetime
from kalman_filter import create_wind_speed_filter

# ==============================================================================
# 滤波器选择 - 在此切换不同的滤波器
# ==============================================================================
# 可选滤波器类型:
#   "Original"        - 原始滤波器 (create_wind_speed_filter)
#   "StrongSmoothing" - 强化平滑滤波器 (平滑度最高，响应较慢)
#   "DualStage"       - 双级滤波器 (综合优化)
#   "RobustOutlier"   - 异常值剔除滤波器 (抗突变干扰)
FILTER_TYPE = "RobustOutlier"


if FILTER_TYPE == "StrongSmoothing":
    from kalman_filter import StrongSmoothingFilter as WindFilter
elif FILTER_TYPE == "DualStage":
    from kalman_filter import DualStageFilter as WindFilter
elif FILTER_TYPE == "RobustOutlier":
    from kalman_filter import RobustOutlierFilter as WindFilter
else:
    # Original - 使用原始函数
    WindFilter = None
# ==============================================================================
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
        self.RTD_DEVICE_IP = "192.168.1.101"
        self.RTD_DEVICE_PORT = 8234
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

        # Kalman filters for wind speeds - 根据 FILTER_TYPE 选择滤波器
        if FILTER_TYPE == "Original" or WindFilter is None:
            self.wind_filters = [create_wind_speed_filter() for _ in range(4)]
        else:
            self.wind_filters = [WindFilter() for _ in range(4)]

        # 压力滤波器（平滑压力数据，消除K因子跳变）
        from kalman_filter import create_pressure_filter
        self.pressure_filter = create_pressure_filter(101.325)

        # RTD温度滤波器（平滑RTD温度，消除K因子跳变）
        from kalman_filter import create_temperature_filter
        self.rtd_filters = [create_temperature_filter(23.1) for _ in range(4)]

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

        # Data log file (每次启动时清除旧内容)
        self.log_file_path = Path("wind_data_log.csv")
        self._init_log_file()

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
        titles = ['Wind Sensor 1 (RTD05)', 'Wind Sensor 2 (RTD06)',
                 'Wind Sensor 3 (RTD07)', 'Wind Sensor 4 (RTD08)']

        for i, ax in enumerate(self.axes.flat):
            ax.set_title(titles[i], fontsize=12)
            ax.set_xlabel('Time (s)', fontsize=10)
            ax.set_ylabel('Wind Speed (m/s)', fontsize=10)
            ax.grid(True, alpha=0.3)

            # Create line objects
            line_raw, = ax.plot([], [], color=colors[i], alpha=0.4, linewidth=1, label='Raw')
            line_filtered, = ax.plot([], [], color=colors[i], linewidth=1, label='Filtered')
            line_corrected, = ax.plot([], [], color='red', linewidth=1, label='Density Corrected')

            self.lines_raw.append(line_raw)
            self.lines_filtered.append(line_filtered)
            self.lines_corrected.append(line_corrected)

            # Add legend at lower right
            ax.legend(loc='lower right')

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

    def _init_log_file(self):
        """初始化日志文件，每次启动时清除旧内容"""
        try:
            with open(self.log_file_path, 'w', encoding='utf-8') as f:
                # 写入CSV表头
                header = "Timestamp,Time(s),Sensor,Raw(m/s),Filtered(m/s),Corrected(m/s),RTD(C),Pressure(kPa),Humidity(%RH),Density(kg/m3),K_factor_raw,K_factor_filtered\n"
                f.write(header)
            print(f"日志文件已初始化: {self.log_file_path.absolute()}")
        except Exception as e:
            print(f"日志文件初始化失败: {e}")

    def _write_log(self, current_time, sensor_idx, raw, filtered, corrected, rtd_temp, pressure, humidity, density, k_factor_raw, k_factor_filtered):
        """写入数据到日志文件"""
        try:
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            with open(self.log_file_path, 'a', encoding='utf-8') as f:
                line = f"{timestamp},{current_time:.3f},{sensor_idx+1},{raw:.4f},{filtered:.4f},{corrected:.4f},{rtd_temp:.2f},{pressure:.3f},{humidity:.2f},{density:.4f},{k_factor_raw:.6f},{k_factor_filtered:.6f}\n"
                f.write(line)
        except Exception as e:
            # 静默失败，不影响主程序
            pass

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
            print(f"Successfully connected to wind device {self.WIND_DEVICE_IP}:{self.WIND_DEVICE_PORT}")
        except Exception as e:
            print(f"Wind device connection failed: {str(e)}")
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
            print(f"Successfully connected to RTD device {self.RTD_DEVICE_IP}:{self.RTD_DEVICE_PORT}")
        except Exception as e:
            print(f"RTD device connection failed: {str(e)}")
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
                    raise socket.timeout("Receive timeout")
                time.sleep(0.01)

            # Parse response
            parsed_data = parse_rtu_response(response_bytes)
            if "error" in parsed_data:
                print(f"Wind data parse failed: {parsed_data['error']}")
                return None

            registers = parsed_data["registers"]
            if len(registers) < self.WIND_REG_COUNT:
                print(f"Insufficient wind data: need {self.WIND_REG_COUNT}, got {len(registers)}")
                return None

            # Extract wind speed data (registers 4-7)
            wind_speeds_raw = []
            for i in range(4, 8):
                raw_value = registers[i]
                current_value = raw_value / 249  # Convert to current (mA)
                wind_speed_raw = (current_value - 4) * 30 / 16  # Convert to wind speed (m/s)
                wind_speeds_raw.append(wind_speed_raw)

            # Extract pressure (register 1)
            pressure_raw = registers[1]
            pressure_current = pressure_raw / 249
            pressure = (pressure_current - 4) * 7.5

            # Use fixed humidity value 40% (according to sensor.py)
            humidity = 40.0

            return wind_speeds_raw, pressure, humidity

        except Exception as e:
            print(f"Wind data read failed: {str(e)}")
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
                    raise socket.timeout("RTD receive timeout")
                time.sleep(0.01)

            # Parse response
            parsed_data = parse_rtu_response(response_bytes)
            if "error" in parsed_data:
                print(f"RTD data parse failed: {parsed_data['error']}")
                return None

            registers = parsed_data["registers"]
            if len(registers) < self.RTD_REG_COUNT:
                print(f"Insufficient RTD data: need {self.RTD_REG_COUNT}, got {len(registers)}")
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
            print(f"RTD data read failed: {str(e)}")
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
            pressure_raw = wind_result[1] if len(wind_result) > 1 else self.CALIBRATION_PRESSURE
            humidity = wind_result[2] if len(wind_result) > 2 else self.CALIBRATION_RH * 100
            rtd_temps_raw = rtd_result

            # 对压力和RTD温度进行滤波，平滑K因子
            pressure = self.pressure_filter.update(pressure_raw)

            # Add timestamp
            self.time_data.append(current_time)

            # Update data storage
            for i in range(4):
                # Store raw wind speed
                self.wind_raw_data[i].append(wind_speeds_raw[i])

                # Apply Kalman filter
                wind_speed_filtered = self.wind_filters[i].update(wind_speeds_raw[i])
                self.wind_filtered_data[i].append(wind_speed_filtered)

                # 对RTD温度进行滤波
                rtd_temp_filtered = self.rtd_filters[i].update(rtd_temps_raw[i])

                # Store RTD temperature (原始值，用于显示)
                self.rtd_temp_data[i].append(rtd_temps_raw[i])

                # 计算原始K因子（用原始RTD温度和原始压力）
                density_raw = calculate_air_density(rtd_temps_raw[i], pressure_raw, humidity)
                K_raw = (self.calibration_density / density_raw) ** 0.5 if density_raw > 0 else 1.0

                # Calculate air density using 滤波后的RTD温度和压力
                density = calculate_air_density(rtd_temp_filtered, pressure, humidity)

                # Calculate K factor and corrected wind speed（使用滤波后的K）
                K_filtered = (self.calibration_density / density) ** 0.5 if density > 0 else 1.0
                wind_speed_corrected = wind_speed_filtered * K_filtered
                self.wind_corrected_data[i].append(wind_speed_corrected)

                # 写入日志文件
                self._write_log(current_time, i, wind_speeds_raw[i], wind_speed_filtered,
                               wind_speed_corrected, rtd_temps_raw[i], pressure, humidity, density, K_raw, K_filtered)

            # Store environmental data (using first sensor's values)
            self.pressure_data.append(pressure)
            self.humidity_data.append(humidity)
            # 使用滤波后的RTD温度计算环境密度
            rtd_temps_filtered = [self.rtd_filters[i].get_filtered_value() for i in range(4)]
            avg_density = calculate_air_density(
                np.mean(rtd_temps_filtered), pressure, humidity
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
                        current_rtd = self.rtd_temp_data[i][-1]  # 显示原始值
                        # 使用滤波后的RTD温度计算密度和K因子
                        current_rtd_filtered = self.rtd_filters[i].get_filtered_value()
                        current_density = calculate_air_density(current_rtd_filtered, pressure, humidity)
                        K = (self.calibration_density / current_density) ** 0.5 if current_density > 0 else 1.0

                        self.current_value_texts[i].set_text(
                            f'Raw: {recent_raw[-1]:.2f} m/s\n'
                            f'Filtered: {recent_filtered[-1]:.2f} m/s\n'
                            f'Corrected: {recent_corrected[-1]:.2f} m/s'
                        )

                        self.density_texts[i].set_text(
                            f'RTD: {current_rtd:.1f}C\n'
                            f'Density: {current_density:.3f} kg/m3\n'
                            f'K factor: {K:.3f}'
                        )

            # Update title with statistics
            success_rate = self.success_count / self.read_count * 100 if self.read_count > 0 else 0
            if len(self.density_data) > 0:
                current_density = self.density_data[-1]
                current_pressure = self.pressure_data[-1]
                current_humidity = self.humidity_data[-1]

                self.fig.suptitle(
                    f'RTD Wind Speed Monitoring | Success: {success_rate:.1f}% | '
                    f'Reads: {self.read_count} | Runtime: {current_time:.0f}s\n'
                    f'Environment: Density={current_density:.3f} kg/m3, Pressure={current_pressure:.1f} kPa, Humidity={current_humidity:.1f}%RH',
                    fontsize=12, fontweight='bold'
                )

            # Print status
            if self.frame_count % 10 == 0:
                print(f"\rTime: {current_time:6.1f}s | "
                      f"Corrected: {self.wind_corrected_data[0][-1]:5.2f} | "
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
        print("RTD Wind Speed Monitoring System with Air Density Correction")
        print("="*60)
        print(f"Wind device: {self.WIND_DEVICE_IP}:{self.WIND_DEVICE_PORT}")
        print(f"RTD device: {self.RTD_DEVICE_IP}:{self.RTD_DEVICE_PORT}")
        print(f"Calibration: {self.CALIBRATION_TEMP}°C, {self.CALIBRATION_RH*100}%RH, {self.CALIBRATION_PRESSURE}kPa")
        print(f"Calibration density: {self.calibration_density:.3f} kg/m³")
        print("Display: Raw, Kalman filtered, and density corrected values")
        print("Press Ctrl+C to stop")
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
            print("\n\nUser interrupted, stopping...")
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
            print("Statistics Report")
            print("="*60)
            print(f"Total runtime: {total_runtime:.1f} seconds")
            print(f"Total reads: {self.read_count}")
            print(f"Successful reads: {self.success_count}")
            print(f"Failed reads: {self.fail_count}")
            print(f"Success rate: {success_rate:.1f}%")
            print(f"Data points stored: {len(self.time_data)}")
            print("="*60)

if __name__ == "__main__":
    plotter = RTDWindDensityPlotter()
    plotter.start()