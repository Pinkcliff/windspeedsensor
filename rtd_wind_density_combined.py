"""
RTD Wind Speed Monitoring with Air Density Correction - Combined Plot
All 4 wind sensors on a single plot with air density correction
"""

import socket
import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import CheckButtons
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

class RTDWindDensityCombined:
    def __init__(self):
        # Wind sensor device parameters
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

        # Data storage
        self.max_points = 2000
        self.time_data = deque()

        # Wind speed data series for 4 sensors
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

        # Setup matplotlib - Single plot with checkboxes
        plt.style.use('default')
        # Create figure with space for checkboxes
        self.fig = plt.figure(figsize=(16, 8))
        # Main plot takes left part
        self.ax = plt.subplot2grid((4, 10), (0, 0), colspan=7, rowspan=4)

        self.fig.suptitle('RTD Wind Speed Monitoring with Air Density Correction - Combined View\n'
                         f'Calibration: {self.CALIBRATION_TEMP}°C, {self.CALIBRATION_RH*100}%RH, {self.CALIBRATION_PRESSURE}kPa',
                         fontsize=14, fontweight='bold')

        # Initialize plot elements for all sensors
        self.lines_raw = []
        self.lines_filtered = []
        self.lines_corrected = []

        # Colors and labels for 4 sensors
        self.colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728']
        self.sensor_labels = ['Sensor 1 (RTD05)', 'Sensor 2 (RTD06)',
                             'Sensor 3 (RTD07)', 'Sensor 4 (RTD08)']

        # Create line objects for each sensor
        for i in range(4):
            # Raw data (dashed line)
            line_raw, = self.ax.plot([], [],
                                    color=self.colors[i],
                                    alpha=0.4,
                                    linewidth=1,
                                    linestyle='--',
                                    label=f'{self.sensor_labels[i]} Raw')

            # Filtered data (solid line)
            line_filtered, = self.ax.plot([], [],
                                         color=self.colors[i],
                                         linewidth=2,
                                         label=f'{self.sensor_labels[i]} Filtered')

            # Corrected data (thick solid line)
            line_corrected, = self.ax.plot([], [],
                                          color='red',
                                          linewidth=2.5,
                                          alpha=0.7,
                                          label=f'{self.sensor_labels[i]} Corrected')

            self.lines_raw.append(line_raw)
            self.lines_filtered.append(line_filtered)
            self.lines_corrected.append(line_corrected)

        # Setup axes
        self.ax.set_xlabel('Time (s)', fontsize=12)
        self.ax.set_ylabel('Wind Speed (m/s)', fontsize=12)
        self.ax.grid(True, alpha=0.3)

        # Set initial y-axis range
        self.ax.set_ylim(-2, 25)

        # Create checkboxes
        # Checkbox area takes right part
        checkbox_ax = plt.subplot2grid((4, 10), (0, 8), colspan=2, rowspan=4)
        checkbox_ax.axis('off')

        # Prepare labels for checkboxes
        labels = []
        for i in range(4):
            labels.append(f'{self.sensor_labels[i]} Raw')
            labels.append(f'{self.sensor_labels[i]} Filtered')
            labels.append(f'{self.sensor_labels[i]} Corrected')

        # Create checkbox
        self.check = CheckButtons(checkbox_ax, labels,
                                  [True] * 12)  # All checked by default

        # Connect checkbox to function
        self.check.on_clicked(self.update_visibility)

        # Make all lines accessible by name
        self.all_lines = []
        for i in range(4):
            self.all_lines.append(self.lines_raw[i])
            self.all_lines.append(self.lines_filtered[i])
            self.all_lines.append(self.lines_corrected[i])

        # Connect to devices
        self.connect_devices()

    def update_visibility(self, label):
        """Update visibility of lines based on checkbox state"""
        # Get the index of the clicked label
        # CheckButtons stores labels as Text objects
        for i, text_obj in enumerate(self.check.labels):
            if text_obj.get_text() == label:
                index = i
                break
        else:
            return  # Label not found

        # Toggle visibility
        if self.check.get_status()[index]:
            self.all_lines[index].set_visible(True)
        else:
            self.all_lines[index].set_visible(False)

        # Redraw
        self.fig.canvas.draw_idle()

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
            pressure = wind_result[1] if len(wind_result) > 1 else self.CALIBRATION_PRESSURE
            humidity = wind_result[2] if len(wind_result) > 2 else self.CALIBRATION_RH * 100
            rtd_temps = rtd_result

            # Add timestamp
            self.time_data.append(current_time)

            # Update data storage for all sensors
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

            # Update all plots
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
                self.ax.set_xlim(x_min, x_max)
            else:
                self.ax.set_xlim(0, max(time_window, current_time + time_margin))

            # Dynamic y-axis adjustment for all data
            all_recent_data = []
            for i in range(4):
                if len(self.wind_raw_data[i]) > 0:
                    recent_points = min(100, len(self.wind_raw_data[i]))
                    all_recent_data.extend(list(self.wind_raw_data[i])[-recent_points:])
                    all_recent_data.extend(list(self.wind_filtered_data[i])[-recent_points:])
                    all_recent_data.extend(list(self.wind_corrected_data[i])[-recent_points:])

            if all_recent_data:
                y_min = min(all_recent_data)
                y_max = max(all_recent_data)

                # Add padding
                y_range = y_max - y_min
                if y_range < 2:
                    y_range = 2
                y_min -= y_range * 0.1
                y_max += y_range * 0.1

                # Ensure minimum values
                if y_min < 0: y_min = 0
                if y_max > 30: y_max = 30

                self.ax.set_ylim(y_min, y_max)

            # Update title with statistics
            success_rate = self.success_count / self.read_count * 100 if self.read_count > 0 else 0
            if len(self.density_data) > 0:
                current_density = self.density_data[-1]
                current_pressure = self.pressure_data[-1]
                current_humidity = self.humidity_data[-1]

                self.fig.suptitle(
                    f'RTD Wind Speed Monitoring - Combined View | Success: {success_rate:.1f}% | '
                    f'Reads: {self.read_count} | Runtime: {current_time:.0f}s\n'
                    f'Environment: Density={current_density:.3f} kg/m3, Pressure={current_pressure:.1f} kPa, Humidity={current_humidity:.1f}%RH',
                    fontsize=12, fontweight='bold'
                )

            # Print status
            if self.frame_count % 10 == 0:
                status_str = f"\rTime: {current_time:6.1f}s | Corrected: "
                for i in range(4):
                    status_str += f"S{i+1}:{self.wind_corrected_data[i][-1]:5.2f}m/s "
                print(status_str, end='')
        else:
            self.read_count += 1
            self.fail_count += 1

        return self.lines_raw + self.lines_filtered + self.lines_corrected

    def start(self):
        """Start real-time plotting"""
        print("\n" + "="*60)
        print("RTD Wind Speed Monitoring System - Interactive Plot View")
        print("="*60)
        print(f"Wind device: {self.WIND_DEVICE_IP}:{self.WIND_DEVICE_PORT}")
        print(f"RTD device: {self.RTD_DEVICE_IP}:{self.RTD_DEVICE_PORT}")
        print(f"Calibration: {self.CALIBRATION_TEMP}°C, {self.CALIBRATION_RH*100}%RH, {self.CALIBRATION_PRESSURE}kPa")
        print(f"Calibration density: {self.calibration_density:.3f} kg/m³")
        print("Display: All 4 sensors on single plot with interactive checkboxes")
        print("  - Dashed lines: Raw data")
        print("  - Colored solid lines: Filtered data")
        print("  - Red solid lines: Density corrected data")
        print("\nUse checkboxes on the right to toggle visibility of each data line")
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
    plotter = RTDWindDensityCombined()
    plotter.start()