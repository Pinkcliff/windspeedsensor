"""
Real-time Wind Speed Plotter
Display original and filtered values from 4 wind speed sensors
"""

import socket
import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
from kalman_filter import create_wind_speed_filter
from typing import List, Optional

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


class WindSpeedPlotter:
    def __init__(self):
        # Device parameters
        self.DEVICE_IP = "192.168.0.101"
        self.DEVICE_PORT = 8234
        self.SLAVE_ADDR = 1
        self.FUNC_CODE = 0x04
        self.START_REG = 0
        self.REG_COUNT = 8
        self.TIMEOUT = 5
        self.BUFFER_SIZE = 1024

        # Data storage - using deque with unlimited size to prevent data loss
        self.max_points = 2000  # Store more data points to prevent disappearance
        self.time_data = deque()

        # Raw wind speed data
        self.wind_raw_data = [
            deque(),  # Wind speed 1
            deque(),  # Wind speed 2
            deque(),  # Wind speed 3
            deque()   # Wind speed 4
        ]

        # Filtered wind speed data
        self.wind_filtered_data = [
            deque(),  # Wind speed 1
            deque(),  # Wind speed 2
            deque(),  # Wind speed 3
            deque()   # Wind speed 4
        ]

        # Kalman filters
        self.wind_filters = [create_wind_speed_filter() for _ in range(4)]

        # Connection object
        self.sock: Optional[socket.socket] = None
        self.connected = False

        # Statistics
        self.read_count = 0
        self.success_count = 0
        self.fail_count = 0
        self.start_time = time.time()

        # Setup matplotlib
        plt.style.use('default')
        self.fig, self.axes = plt.subplots(2, 2, figsize=(14, 10))
        self.fig.suptitle('Real-time Wind Speed Monitoring (Raw vs Kalman Filtered)',
                         fontsize=16, fontweight='bold')

        # Adjust subplot spacing
        plt.subplots_adjust(hspace=0.3, wspace=0.25)

        # Initialize 4 subplots
        self.lines_raw = []
        self.lines_filtered = []
        self.current_value_texts = []  # Initialize list for current value text objects
        colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728']

        for i, ax in enumerate(self.axes.flat):
            # Set title and labels
            ax.set_title(f'Wind Speed Sensor {i+1}', fontsize=12)
            ax.set_xlabel('Time (seconds)', fontsize=10)
            ax.set_ylabel('Wind Speed (m/s)', fontsize=10)
            ax.grid(True, alpha=0.3)

            # Create line objects
            line_raw, = ax.plot([], [], color=colors[i], alpha=0.5, linewidth=1, label='Raw')
            line_filtered, = ax.plot([], [], color=colors[i], linewidth=2, label='Filtered')

            self.lines_raw.append(line_raw)
            self.lines_filtered.append(line_filtered)

            # Add legend
            ax.legend(loc='upper right')

            # Initialize text objects for current values
            text_obj = ax.text(0.02, 0.95, '', transform=ax.transAxes,
                            verticalalignment='top', fontsize=10,
                            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
            self.current_value_texts.append(text_obj)

            # Set initial y-axis range
            ax.set_ylim(-1, 25)

        # Start connection
        self.connect_device()

    def connect_device(self) -> bool:
        """Connect to device"""
        try:
            if self.sock:
                self.sock.close()

            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(self.TIMEOUT)
            self.sock.connect((self.DEVICE_IP, self.DEVICE_PORT))
            self.connected = True
            print(f"Successfully connected to device {self.DEVICE_IP}:{self.DEVICE_PORT}")
            return True
        except Exception as e:
            print(f"Connection failed: {str(e)}")
            self.connected = False
            return False

    def read_wind_data(self):
        """Read wind speed data"""
        if not self.connected or not self.sock:
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
            self.sock.sendall(request)

            # Receive response
            response_bytes = b""
            request_start_time = time.time()

            while True:
                chunk = self.sock.recv(self.BUFFER_SIZE)
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
                print(f"Parse failed: {parsed_data['error']}")
                return None

            registers = parsed_data["registers"]
            if len(registers) < self.REG_COUNT:
                print(f"Insufficient data: need {self.REG_COUNT}, got {len(registers)}")
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

            return wind_speeds

        except Exception as e:
            print(f"Read failed: {str(e)}")
            # Try to reconnect
            if "Connection" in str(e) or "reset" in str(e):
                self.connected = False
                self.connect_device()
            return None

    def update(self, frame):
        """Update data"""
        # Track frame count
        self.frame_count = getattr(self, 'frame_count', 0) + 1
        current_time = time.time() - self.start_time

        # Read data
        wind_data = self.read_wind_data()

        if wind_data:
            self.read_count += 1
            self.success_count += 1

            # Add timestamp
            self.time_data.append(current_time)

            # Update data
            for i, (raw_val, filtered_val) in enumerate(wind_data):
                self.wind_raw_data[i].append(raw_val)
                self.wind_filtered_data[i].append(filtered_val)

            # Update plots
            for i in range(4):
                if len(self.time_data) > 0:
                    # Update raw data line
                    self.lines_raw[i].set_data(self.time_data, self.wind_raw_data[i])
                    # Update filtered data line
                    self.lines_filtered[i].set_data(self.time_data, self.wind_filtered_data[i])

                    # Dynamic x-axis adjustment - show all data or last 60 seconds
                    if current_time > 60 and len(self.time_data) > 100:
                        # Show last 60 seconds
                        self.axes.flat[i].set_xlim(current_time - 60, current_time)
                    else:
                        # Show all data
                        self.axes.flat[i].set_xlim(0, max(60, current_time))

                    # Dynamic y-axis adjustment - ALWAYS update to show current data properly
                    if len(self.wind_raw_data[i]) > 0 and len(self.wind_filtered_data[i]) > 0:
                        # Get the most recent data points for y-axis range
                        recent_points = min(100, len(self.wind_raw_data[i]))  # Use last 100 points
                        recent_raw = list(self.wind_raw_data[i])[-recent_points:]
                        recent_filtered = list(self.wind_filtered_data[i])[-recent_points:]

                        if recent_raw and recent_filtered:
                            all_data = recent_raw + recent_filtered
                            y_min = min(all_data)
                            y_max = max(all_data)

                            # Add padding
                            y_range = y_max - y_min
                            if y_range < 2:  # Minimum range of 2 m/s
                                y_range = 2
                            y_min -= y_range * 0.1  # 10% padding
                            y_max += y_range * 0.1

                            # Ensure minimum values
                            if y_min < 0: y_min = 0
                            if y_max > 30: y_max = 30

                            # IMPORTANT: Disable autoscale before setting limits
                            self.axes.flat[i].set_autoscale_on(False)

                            # Force y-axis update with multiple approaches
                            ax = self.axes.flat[i]

                            # Disable autoscale
                            ax.set_autoscale_on(False)

                            # Clear and redraw the axis (more drastic approach)
                            # ax.clear()
                            # ax.relim()

                            # Set the limits multiple ways
                            ax.set_ylim(y_min, y_max)
                            ax.set_ybound(lower=y_min, upper=y_max)

                            # Force the figure to redraw
                            if self.frame_count % 10 == 0:  # Every 10 frames
                                self.fig.canvas.draw_idle()
                                self.fig.canvas.flush_events()

                            # Debug print
                            if i == 0 and self.frame_count % 20 == 0:  # Less frequent printing
                                print(f"\n[Frame {self.frame_count}] Y-axis Sensor {i+1}: [{y_min:.2f}, {y_max:.2f}]", end='')

                        # Update current value text
                        if recent_raw and recent_filtered:
                            current_raw = recent_raw[-1]
                            current_filtered = recent_filtered[-1]
                            self.current_value_texts[i].set_text(
                                f'Raw: {current_raw:.2f} m/s\n'
                                f'Filtered: {current_filtered:.2f} m/s\n'
                                f'Diff: {abs(current_raw - current_filtered):.2f}'
                            )

            # Update success information
            success_rate = self.success_count / self.read_count * 100 if self.read_count > 0 else 0
            self.fig.suptitle(
                f'Real-time Wind Speed Monitoring | Success Rate: {success_rate:.1f}% | '
                f'Reads: {self.read_count} | Runtime: {current_time:.0f}s',
                fontsize=14, fontweight='bold'
            )

            # Print latest data
            print(f"\rTime: {current_time:6.1f}s | "
                  f"Wind: {wind_data[0][1]:5.2f} | {wind_data[1][1]:5.2f} | "
                  f"{wind_data[2][1]:5.2f} | {wind_data[3][1]:5.2f} m/s", end='')
        else:
            self.read_count += 1
            self.fail_count += 1

        return self.lines_raw + self.lines_filtered

    def start(self):
        """Start real-time plotting"""
        print("\n" + "="*60)
        print("Real-time Wind Speed Monitoring System")
        print("="*60)
        print(f"Device address: {self.DEVICE_IP}:{self.DEVICE_PORT}")
        print(f"Display: Raw and Kalman filtered values from 4 wind speed sensors")
        print(f"Data points: All data stored (no automatic deletion)")
        print(f"Time window: Shows last 60 seconds of data")
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
            if self.sock:
                self.sock.close()

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
    plotter = WindSpeedPlotter()
    plotter.start()