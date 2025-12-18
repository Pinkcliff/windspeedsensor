"""
Wind Speed and Air Density Monitoring System
Displays raw, filtered, and corrected values from 4 wind speed sensors
with RTD temperature-based air density correction
"""

import socket
import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
from kalman_filter import create_wind_speed_filter
from typing import List, Optional
import csv
import os
from datetime import datetime
from Refrigerant import AIR

# Modbus RTU frame processing functions
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


class WindSpeedDensityPlotter:
    def __init__(self):
        # Calibration parameters
        self.CALIBRATION_TEMP = 23.1
        self.CALIBRATION_RH = 0.65
        self.CALIBRATION_PRESSURE = 101.325
        self.calibration_density = self.calculate_air_density(
            self.CALIBRATION_TEMP,
            self.CALIBRATION_PRESSURE,
            self.CALIBRATION_RH * 100
        )

        # Device parameters for analog sensors
        self.DEVICE_IP = "192.168.0.101"
        self.DEVICE_PORT = 8234
        self.SLAVE_ADDR = 1
        self.FUNC_CODE = 0x04
        self.START_REG = 0
        self.REG_COUNT = 12
        self.TIMEOUT = 5
        self.BUFFER_SIZE = 1024

        # Device parameters for RTD sensors
        self.RTD_IP = "192.168.1.101"
        self.RTD_PORT = 8234
        self.RTD_SLAVE_ADDR = 1
        self.RTD_FUNC_CODE = 0x04
        self.RTD_START_REG = 0
        self.RTD_REG_COUNT = 12

        # Data storage - using deque with unlimited size
        self.max_points = 2000
        self.time_data = deque()

        # Wind speed data
        self.wind_raw_data = [
            deque(),  # Wind speed 1 (analog)
            deque(),  # Wind speed 2 (analog)
            deque(),  # Wind speed 3 (analog)
            deque()   # Wind speed 4 (analog)
        ]

        # Filtered wind speed data
        self.wind_filtered_data = [
            deque(),  # Wind speed 1 (analog)
            deque(),  # Wind speed 2 (analog)
            deque(),  # Wind speed 3 (analog)
            deque()   # Wind speed 4 (analog)
        ]

        # Corrected wind speed data
        self.wind_corrected_data = [
            deque(),  # Wind speed 1
            deque(),  # Wind speed 2
            deque(),  # Wind speed 3
            deque()   # Wind speed 4
        ]

        # RTD temperature data
        self.rtd_temps = [0.0] * 4  # Store 4 RTD temperatures

        # Kalman filters for analog wind speed
        self.wind_filters = [create_wind_speed_filter() for _ in range(4)]

        # Connection objects
        self.analog_sock: Optional[socket.socket] = None
        self.rtd_sock: Optional[socket.socket] = None
        self.analog_connected = False
        self.rtd_connected = False

        # Statistics
        self.read_count = 0
        self.success_count = 0
        self.fail_count = 0
        self.start_time = time.time()

        # Data file setup
        self.setup_data_file()

        # Setup matplotlib
        plt.style.use('default')
        self.fig, self.axes = plt.subplots(2, 2, figsize=(14, 10))
        self.fig.suptitle(
            f'Real-time Wind Speed Monitoring with Air Density Correction\n'
            f'Calibration: T={self.CALIBRATION_TEMP}℃, RH={self.CALIBRATION_RH*100:.0f}%, '
            f'P={self.CALIBRATION_PRESSURE:.3f}kPa (ρ={self.calibration_density:.3f}kg/m³)',
            fontsize=14, fontweight='bold'
        )

        # Adjust subplot spacing
        plt.subplots_adjust(hspace=0.3, wspace=0.25)

        # Initialize 4 subplots
        self.lines_raw = []
        self.lines_filtered = []
        self.lines_corrected = []
        self.current_value_texts = []
        colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728']

        for i, ax in enumerate(self.axes.flat):
            # Set title and labels
            ax.set_title(f'Sensor {i+1} (Analog) → RTD{i+5}', fontsize=12)
            ax.set_xlabel('Time (seconds)', fontsize=10)
            ax.set_ylabel('Wind Speed (m/s)', fontsize=10)
            ax.grid(True, alpha=0.3)

            # Create line objects
            line_raw, = ax.plot([], [], color=colors[i], alpha=0.5, linewidth=1, label='Raw')
            line_filtered, = ax.plot([], [], color=colors[i], linewidth=2, label='Filtered')
            line_corrected, = ax.plot([], [], 'green', linewidth=2.5, label='Corrected')

            self.lines_raw.append(line_raw)
            self.lines_filtered.append(line_filtered)
            self.lines_corrected.append(line_corrected)

            # Add legend
            ax.legend(loc='upper right')

            # Initialize text objects for current values
            text_obj = ax.text(0.02, 0.95, '', transform=ax.transAxes,
                                verticalalignment='top', fontsize=10,
                                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
            self.current_value_texts.append(text_obj)

            # Set initial y-axis range
            ax.set_ylim(-2, 15)

        # Start connections
        self.connect_analog_device()
        self.connect_rtd_device()

    def calculate_air_density(self, temperature, pressure, humidity):
        """Calculate air density"""
        try:
            air = AIR(dP=pressure, unit='c', dTdb=temperature, dRh=humidity/100)
            air.updateData()
            prop = air.getProp(unit='c')
            return prop['Density(kg/m3)']
        except Exception as e:
            print(f"Air density calculation error: {e}")
            return 0.0

    def setup_data_file(self):
        """Setup data file for saving data"""
        # Create data directory if it doesn't exist
        self.data_dir = "wind_density_data"
        if not os.path.exists(self.data_dir):
            os.makedirs(self.data_dir)
            print(f"Created directory: {self.data_dir}")

        # Generate filename with timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.data_filename = os.path.join(self.data_dir, f"{timestamp}_wind_density_data.csv")

        # Open file and write header
        self.data_file = open(self.data_filename, 'w', newline='')
        self.csv_writer = csv.writer(self.data_file)

        # Write header
        header = [
            'Timestamp', 'Time_Elapsed_s',
            'Sensor1_Raw', 'Sensor1_Filtered', 'Sensor1_Corrected', 'Sensor1_RTD_Temp',
            'Sensor2_Raw', 'Sensor2_Filtered', 'Sensor2_Corrected', 'Sensor2_RTD_Temp',
            'Sensor3_Raw', 'Sensor3_Filtered', 'Sensor3_Corrected', 'Sensor3_RTD_Temp',
            'Sensor4_Raw', 'Sensor4_Filtered', 'Sensor4_Corrected', 'Sensor4_RTD_Temp',
            'Temperature', 'Pressure', 'Humidity', 'Air_Density', 'Success_Flag'
        ]
        self.csv_writer.writerow(header)
        self.data_file.flush()

        print(f"Data will be saved to: {self.data_filename}")

    def save_data_to_file(self, wind_data, success=True):
        """Save wind speed and density data to CSV file"""
        try:
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            time_elapsed = time.time() - self.start_time

            row = [timestamp, time_elapsed]

            # Add wind data
            for i in range(4):
                if wind_data and len(wind_data) > i:
                    row.extend([
                        wind_data[i]['raw'] if wind_data else '',
                        wind_data[i]['filtered'] if wind_data else '',
                        wind_data[i]['corrected'] if wind_data else '',
                        self.rtd_temps[i]
                    ])
                else:
                    # No data - fill with empty values
                    row.extend([''] * 4)

            # Add environmental data
            row.extend([
                '',  # Temperature (analog)
                '',  # Pressure (analog)
                '',  # Humidity (analog)
                '',  # Air density (using RTD avg)
            ])

            # Add success flag
            row.append('1' if success else '0')

            # Write to file
            self.csv_writer.writerow(row)
            self.data_file.flush()

        except Exception as e:
            print(f"Error saving data: {e}")

    def close_data_file(self):
        """Close the data file"""
        if hasattr(self, 'data_file') and self.data_file:
            self.data_file.close()
            print(f"\nData file closed: {self.data_filename}")
            print(f"Total data points saved: {self.read_count}")

    def connect_analog_device(self) -> bool:
        """Connect to analog sensor device"""
        try:
            if self.analog_sock:
                self.analog_sock.close()

            self.analog_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.analog_sock.settimeout(self.TIMEOUT)
            self.analog_sock.connect((self.DEVICE_IP, self.DEVICE_PORT))
            self.analog_connected = True
            print(f"Successfully connected to analog device {self.DEVICE_IP}:{self.DEVICE_PORT}")
            return True
        except Exception as e:
            print(f"Analog device connection failed: {str(e)}")
            self.analog_connected = False
            return False

    def connect_rtd_device(self) -> bool:
        """Connect to RTD temperature sensor device"""
        try:
            if self.rtd_sock:
                self.rtd_sock.close()

            self.rtd_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.rtd_sock.settimeout(self.TIMEOUT)
            self.rtd_sock.connect((self.RTD_IP, self.RTD_PORT))
            self.rtd_connected = True
            print(f"Successfully connected to RTD device {self.RTD_IP}:{self.RTD_PORT}")
            return True
        except Exception as e:
            print(f"RTD device connection failed: {str(e)}")
            self.rtd_connected = False
            return False

    def read_analog_data(self):
        """Read analog sensor data"""
        if not self.analog_connected or not self.analog_sock:
            if not self.connect_analog_device():
                return None

        try:
            # Build request
            request = build_rtu_request(
                slave_addr=self.SLAVE_ADDR,
                start_reg=self.START_REG,
                reg_count=self.REG_COUNT,
                func_code=self.FUNC_CODE
            )

            # Send request
            self.analog_sock.sendall(request)

            # Receive response
            response_bytes = b""
            request_start_time = time.time()

            while True:
                chunk = self.analog_sock.recv(self.BUFFER_SIZE)
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
                print(f"Analog parse failed: {parsed_data['error']}")
                return None

            registers = parsed_data["registers"]
            if len(registers) < self.REG_COUNT:
                print(f"Analog insufficient data: need {self.REG_COUNT}, got {len(registers)}")
                return None

            # Extract wind speed data (registers 4-7)
            wind_speeds = []
            for i in range(4, 8):
                raw_value = registers[i]
                current_value = raw_value / 249  # Convert to current (mA)
                wind_speed_raw = (current_value - 4) * 30 / 16  # Convert to wind speed (m/s)

                # Apply Kalman filter
                wind_speed_filtered = self.wind_filters[i-4].update(wind_speed_raw)

                wind_speeds.append((wind_speed_raw, wind_speed_filtered))

            # Extract environmental data for logging
            # Temperature (register 0)
            temp_raw = registers[0]
            temp_current = temp_raw / 249
            temperature = (temp_current - 4) * 7.5 - 40

            # Pressure (register 1)
            pressure_raw = registers[1]
            pressure_current = pressure_raw / 249
            pressure = (pressure_current - 4) * 7.5

            # Humidity (register 10)
            humidity = 0.0
            if len(registers) > 10:
                humidity_raw = registers[10]
                humidity_current = humidity_raw / 249
                humidity = (humidity_current - 4) * 100 / 16

            return wind_speeds, temperature, pressure, humidity

        except Exception as e:
            print(f"Analog read failed: {str(e)}")
            if "Connection" in str(e) or "reset" in str(e):
                self.analog_connected = False
                self.connect_analog_device()
            return None

    def read_rtd_data(self):
        """Read RTD temperature data"""
        if not self.rtd_connected or not self.rtd_sock:
            if not self.connect_rtd_device():
                return None

        try:
            # Build request
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
                    raise socket.timeout("Receive timeout")
                time.sleep(0.01)

            # Parse response
            parsed_data = parse_rtu_response(response_bytes)
            if "error" in parsed_data:
                print(f"RTD parse failed: {parsed_data['error']}")
                return None

            registers = parsed_data["registers"]
            if len(registers) < self.RTD_REG_COUNT:
                print(f"RTD insufficient data: need {self.RTD_REG_COUNT}, got {len(registers)}")
                return None

            # Extract RTD temperature data (registers 4-7 for sensors 5-8)
            rtd_temps = []
            for i in range(4, 8):
                temp_raw = registers[i]
                temperature = temp_raw / 10  # RTD conversion formula
                rtd_temps.append(temperature)

            return rtd_temps

        except Exception as e:
            print(f"RTD read failed: {str(e)}")
            if "Connection" in str(e) or "reset" in str(e):
                self.rtd_connected = False
                self.connect_rtd_device()
            return None

    def update(self, frame):
        """Update data and plots"""
        # Track frame count
        self.frame_count = getattr(self, 'frame_count', 0) + 1
        current_time = time.time() - self.start_time

        # Read data
        analog_result = self.read_analog_data()
        rtd_result = self.read_rtd_data()

        if analog_result and rtd_result:
            self.read_count += 1
            self.success_count += 1

            # Unpack data
            wind_speeds = analog_result[0]
            temperature = analog_result[1] if len(analog_result) > 1 else None
            pressure = analog_result[2] if len(analog_result) > 2 else None
            humidity = analog_result[3] if len(analog_result) > 3 else None

            # Update RTD temperatures
            self.rtd_temps = rtd_result

            # Calculate corrected wind speeds
            corrected_speeds = []
            wind_data_dict = []
            for i in range(4):
                # Calculate air density for this RTD
                density = self.calculate_air_density(rtd_temps[i], pressure, humidity)
                K = (self.calibration_density / density) ** 0.5 if density > 0 else 1.0
                corrected_speed = wind_speeds[i][1] * K
                corrected_speeds.append(corrected_speed)

                # Store data for file saving
                wind_data_dict.append({
                    'raw': wind_speeds[i][0],
                    'filtered': wind_speeds[i][1],
                    'corrected': corrected_speed
                })

            # Save data to file
            self.save_data_to_file(wind_data_dict, success=True)

            # Add timestamp
            self.time_data.append(current_time)

            # Update data for plotting
            for i in range(4):
                self.wind_raw_data[i].append(wind_speeds[i][0])
                self.wind_filtered_data[i].append(wind_speeds[i][1])
                self.wind_corrected_data[i].append(corrected_speeds[i])

            # For display purposes
            wind_data = wind_speed_dict
        else:
            self.read_count += 1
            self.fail_count += 1

            # Save failed read attempt
            self.save_data_to_file(None, success=False)
            wind_data = None

        # Update plots
        for i in range(4):
            if len(self.time_data) > 0 and wind_data:
                # Update raw data line
                self.lines_raw[i].set_data(self.time_data, self.wind_raw_data[i])
                # Update filtered data line
                self.lines_filtered[i].set_data(self.time_data, self.wind_filtered_data[i])
                # Update corrected data line
                self.lines_corrected[i].set_data(self.time_data, self.wind_corrected_data[i])

                # Dynamic x-axis adjustment
                time_window = 60  # 60 seconds window
                time_margin = 5

                if current_time > time_window * 0.8 and len(self.time_data) > 100:
                    # Start scrolling
                    x_min = current_time - time_window
                    x_max = current_time + time_margin
                    self.axes.flat[i].set_xlim(x_min, x_max)
                else:
                    # Show all data
                    self.axes.flat[i].set_xlim(0, max(time_window, current_time + time_margin))

                # Dynamic y-axis adjustment
                if (len(self.wind_raw_data[i]) > 0 and len(self.wind_filtered_data[i]) > 0 and
                    len(self.wind_corrected_data[i]) > 0):
                    # Get the most recent data points
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
                        if y_range < 1:
                            y_range = 1
                        y_min -= y_range * 0.1
                        y_max += y_range * 0.1

                        # Ensure minimum values
                        if y_min < -5:
                            y_min = -5
                        if y_max > 20:
                            y_max = 20

                        # Disable autoscale before setting limits
                        self.axes.flat[i].set_autoscale_on(False)
                        self.axes.flat[i].set_ylim(y_min, y_max)

                        # Update current value text
                        current_raw = recent_raw[-1] if recent_raw else 0
                        current_filtered = recent_filtered[-1] if recent_filtered else 0
                        current_corrected = recent_corrected[-1] if recent_corrected else 0
                        rtd_temp = self.rtd_temps[i]

                        # Calculate K value
                        if rtd_temp > 0 and pressure > 0 and humidity > 0:
                            density = self.calculate_air_density(rtd_temp, pressure, humidity)
                            K = (self.calibration_density / density) ** 0.5 if density > 0 else 1.0
                        else:
                            K = 1.0

                        self.current_value_texts[i].set_text(
                            f'Raw: {current_raw:.2f} m/s\n'
                            f'Filtered: {current_filtered:.2f} m/s\n'
                            f'Corrected: {current_corrected:.2f} m/s\n'
                            f'K: {K:.3f}\n'
                            f'RTD{i+5}: {rtd_temp:.1f}℃'
                        )

        # Update title with statistics
        success_rate = self.success_count / self.read_count * 100 if self.read_count > 0 else 0
        self.fig.suptitle(
            f'Wind Speed Monitoring with Air Density Correction | Success: {success_rate:.1f}% | '
            f'Reads: {self.read_count} | Runtime: {current_time:.0f}s',
            fontsize=14, fontweight='bold'
        )

        # Print latest data
        if wind_data:
            print(f"\rTime: {current_time:6.1f}s | "
                  f"Sensors: {wind_data[0]['filtered']:5.2f} | {wind_data[1]['filtered']:5.2f} | "
                  f"{wind_data[2]['filtered']:5.2f} | {wind_data[3]['filtered']:5.2f} m/s", end='')
        else:
            print(f"\rTime: {current_time:6.1f}s | "
                  f"Reading data...  ", end='')

        return self.lines_raw + self.lines_filtered + self.lines_corrected

    def start(self):
        """Start real-time plotting"""
        print("\n" + "="*60)
        print("Wind Speed Monitoring with Air Density Correction")
        print("="*60)
        print(f"Analog device: {self.DEVICE_IP}:{self.DEVICE_PORT}")
        print(f"RTD device: {self.RTD_IP}:{self.RTD_PORT}")
        print(f"Display: Raw, filtered, and density-corrected values from 4 sensors")
        print(f"Calibration: T={self.CALIBRATION_TEMP}℃, RH={self.CALIBRATION_RH*100:.0f}%, P={self.CALIBRATION_PRESSURE:.3f}kPa")
        print("Press Ctrl+C to stop")
        print("="*60)

        # Disable matplotlib's interactive mode to force updates
        plt.ioff()  # Turn off interactive mode

        # Create animation
        self.ani = FuncAnimation(
            self.fig, self.update, interval=100,  # Update every 100ms
            blit=False, cache_frame_data=False, repeat=True
        )

        try:
            plt.show()
        except KeyboardInterrupt:
            print("\n\nUser interrupt, stopping...")
        finally:
            # Close data file
            self.close_data_file()

            # Close socket connections
            if self.analog_sock:
                self.analog_sock.close()
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
    plotter = WindSpeedDensityPlotter()
    plotter.start()