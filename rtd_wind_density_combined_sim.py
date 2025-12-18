"""
RTD Wind Speed Monitoring with Air Density Correction - Combined Interactive Plot
All 4 wind sensors on a single plot with interactive checkboxes - Simulation Version
"""

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import CheckButtons
from collections import deque
import numpy as np
import time
import random
from Refrigerant import AIR
from kalman_filter import create_wind_speed_filter

def calculate_air_density(temperature: float, pressure: float, humidity: float) -> float:
    """Calculate air density"""
    try:
        air = AIR(dP=pressure, unit='c', dTdb=temperature, dRh=humidity)
        air.updateData()
        prop = air.getProp(unit='c')
        return prop['Density(kg/m3)']
    except:
        return 1.195  # Default air density

class RTDWindDensityCombinedSimulator:
    def __init__(self):
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
        self.max_points = 500
        self.time_data = deque()

        # Wind speed data
        self.wind_raw_data = [deque() for _ in range(4)]
        self.wind_filtered_data = [deque() for _ in range(4)]
        self.wind_corrected_data = [deque() for _ in range(4)]

        # RTD temperature data
        self.rtd_temp_data = [deque() for _ in range(4)]

        # Environmental data
        self.pressure_data = deque()
        self.humidity_data = deque()
        self.density_data = deque()

        # Kalman filters
        self.wind_filters = [create_wind_speed_filter() for _ in range(4)]

        # Simulation parameters
        self.base_wind_speed = 5.0
        self.base_rtd_temp = [20.0, 21.0, 19.5, 20.5]
        self.base_pressure = 95.5
        self.base_humidity = 40.0

        # Setup matplotlib with interactive checkboxes
        plt.style.use('default')
        # Create figure with space for checkboxes
        self.fig = plt.figure(figsize=(16, 8))
        # Main plot takes left part
        self.ax = plt.subplot2grid((4, 10), (0, 0), colspan=7, rowspan=4)

        self.fig.suptitle('RTD Wind Speed Monitoring - Interactive Combined View (Simulation)\n'
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

        self.start_time = time.time()

    def update_visibility(self, label):
        """Update visibility of lines based on checkbox state"""
        # Get the index of the clicked label
        index = self.check.labels.index(label)

        # Toggle visibility
        if self.check.get_status()[index]:
            self.all_lines[index].set_visible(True)
        else:
            self.all_lines[index].set_visible(False)

        # Redraw
        self.fig.canvas.draw_idle()

    def generate_simulated_data(self):
        """Generate simulated data"""
        current_time = time.time() - self.start_time

        # Simulate wind speed changes
        wind_speeds_raw = []
        for i in range(4):
            # Different variation for each sensor
            wind_speed = self.base_wind_speed + \
                        2 * np.sin(current_time / 10 + i * np.pi/2) + \
                        random.gauss(0, 0.5) + \
                        i * 0.3  # Offset each sensor
            wind_speed = max(0, wind_speed)  # Ensure non-negative
            wind_speeds_raw.append(wind_speed)

        # Simulate RTD temperature changes
        rtd_temps = []
        for i in range(4):
            temp = self.base_rtd_temp[i] + \
                  2 * np.sin(current_time / 30 + i) + \
                  random.gauss(0, 0.1)
            rtd_temps.append(temp)

        # Simulate pressure and humidity changes
        pressure = self.base_pressure + random.gauss(0, 0.5)
        humidity = self.base_humidity + random.gauss(0, 1.0)

        return wind_speeds_raw, rtd_temps, pressure, humidity

    def update(self, frame):
        """Update data and plots"""
        current_time = time.time() - self.start_time

        # Generate simulated data
        wind_speeds_raw, rtd_temps, pressure, humidity = self.generate_simulated_data()

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

        # Store environmental data
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

        # Dynamic y-axis adjustment for all visible data
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

        # Update title with current values
        if len(self.density_data) > 0:
            current_density = self.density_data[-1]
            current_pressure = self.pressure_data[-1]
            current_humidity = self.humidity_data[-1]

            self.fig.suptitle(
                f'RTD Wind Speed Monitoring - Interactive Combined View (Simulation) | Runtime: {current_time:.0f}s\n'
                f'Environment: Density={current_density:.3f} kg/m3, Pressure={current_pressure:.1f} kPa, Humidity={current_humidity:.1f}%RH',
                fontsize=12, fontweight='bold'
            )

        return self.lines_raw + self.lines_filtered + self.lines_corrected

    def start(self):
        """Start real-time plotting"""
        print("\n" + "="*60)
        print("RTD Wind Speed Monitoring System - Interactive Simulation")
        print("="*60)
        print(f"Calibration: {self.CALIBRATION_TEMP}°C, {self.CALIBRATION_RH*100}%RH, {self.CALIBRATION_PRESSURE}kPa")
        print(f"Calibration density: {self.calibration_density:.3f} kg/m³")
        print("Display: All 4 sensors on single plot with interactive checkboxes")
        print("  - Dashed lines: Raw data")
        print("  - Colored solid lines: Filtered data")
        print("  - Red solid lines: Density corrected data")
        print("\nUse checkboxes on the right to toggle visibility of each data line")
        print("Press Ctrl+C to stop")
        print("="*60)

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
            print("\nProgram stopped")

if __name__ == "__main__":
    print("Starting RTD Wind Speed Interactive Simulator...")
    plotter = RTDWindDensityCombinedSimulator()
    plotter.start()