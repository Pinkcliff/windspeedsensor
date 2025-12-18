"""
RTD Wind Speed Monitoring with Air Density Correction - Interactive Plot
Simplified version with reliable checkbox functionality
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

class RTDWindDensityInteractive:
    def __init__(self, simulation_mode=True):
        self.simulation_mode = simulation_mode

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
        if simulation_mode:
            self.base_wind_speed = 5.0
            self.base_rtd_temp = [20.0, 21.0, 19.5, 20.5]
            self.base_pressure = 95.5
            self.base_humidity = 40.0
            self.start_time = time.time()

        # Setup matplotlib with interactive checkboxes
        plt.style.use('default')
        self.fig = plt.figure(figsize=(16, 8))

        # Main plot takes left part (80% width)
        self.ax = plt.subplot2grid((4, 10), (0, 0), colspan=8, rowspan=4)

        title = "Interactive Combined View (Simulation)" if simulation_mode else "Interactive Combined View"
        self.fig.suptitle(f'RTD Wind Speed Monitoring - {title}\n'
                         f'Calibration: {self.CALIBRATION_TEMP}°C, {self.CALIBRATION_RH*100}%RH, {self.CALIBRATION_PRESSURE}kPa',
                         fontsize=14, fontweight='bold')

        # Colors for 4 sensors
        self.colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728']
        self.sensor_names = ['Sensor 1', 'Sensor 2', 'Sensor 3', 'Sensor 4']

        # Dictionary to store lines by their label
        self.lines = {}

        # Create line objects for each sensor
        for i in range(4):
            # Raw data (dashed line)
            label = f'{self.sensor_names[i]} Raw'
            line, = self.ax.plot([], [], color=self.colors[i], alpha=0.4,
                                linewidth=1, linestyle='--', label=label)
            self.lines[label] = line

            # Filtered data (solid line)
            label = f'{self.sensor_names[i]} Filtered'
            line, = self.ax.plot([], [], color=self.colors[i], linewidth=2, label=label)
            self.lines[label] = line

            # Corrected data (red solid line)
            label = f'{self.sensor_names[i]} Corrected'
            line, = self.ax.plot([], [], color='red', linewidth=2.5, alpha=0.7, label=label)
            self.lines[label] = line

        # Setup axes
        self.ax.set_xlabel('Time (s)', fontsize=12)
        self.ax.set_ylabel('Wind Speed (m/s)', fontsize=12)
        self.ax.grid(True, alpha=0.3)
        self.ax.set_ylim(-2, 25)
        self.ax.set_xlim(0, 60)  # Initial x-axis range

        # Create checkboxes on the right
        checkbox_ax = plt.subplot2grid((4, 10), (0, 8), colspan=2, rowspan=4)
        checkbox_ax.axis('off')

        # Create checkbox with all labels
        self.labels = list(self.lines.keys())
        self.checkbox = CheckButtons(checkbox_ax, self.labels, [True] * len(self.labels))

        # Connect checkbox event
        self.checkbox.on_clicked(self.toggle_line)

        # Set initial visibility
        for i, label in enumerate(self.labels):
            self.lines[label].set_visible(True)

        # Adjust layout
        plt.tight_layout(rect=[0, 0.03, 0.95, 0.96])

    def toggle_line(self, label):
        """Toggle visibility of a line"""
        if label in self.lines:
            # Find the index of this label
            idx = self.labels.index(label)
            # Get the current state
            visible = self.checkbox.get_status()[idx]
            # Set line visibility
            self.lines[label].set_visible(visible)
            # Redraw immediately
            plt.draw()
        else:
            print(f"Warning: Label '{label}' not found in lines")

    def generate_simulated_data(self):
        """Generate simulated data"""
        current_time = time.time() - self.start_time

        # Simulate wind speed changes
        wind_speeds_raw = []
        for i in range(4):
            wind_speed = self.base_wind_speed + \
                        2 * np.sin(current_time / 10 + i * np.pi/2) + \
                        random.gauss(0, 0.5) + \
                        i * 0.3  # Offset each sensor
            wind_speed = max(0, wind_speed)
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
        # Generate simulated data
        wind_speeds_raw, rtd_temps, pressure, humidity = self.generate_simulated_data()
        current_time = time.time() - self.start_time

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

            # Calculate air density and correction
            density = calculate_air_density(rtd_temps[i], pressure, humidity)
            K = (self.calibration_density / density) ** 0.5 if density > 0 else 1.0
            wind_speed_corrected = wind_speed_filtered * K
            self.wind_corrected_data[i].append(wind_speed_corrected)

        # Update line data for all lines (regardless of visibility)
        for i in range(4):
            # Always set data, let visibility control display
            label = f'{self.sensor_names[i]} Raw'
            if label in self.lines:
                self.lines[label].set_data(self.time_data, self.wind_raw_data[i])

            label = f'{self.sensor_names[i]} Filtered'
            if label in self.lines:
                self.lines[label].set_data(self.time_data, self.wind_filtered_data[i])

            label = f'{self.sensor_names[i]} Corrected'
            if label in self.lines:
                self.lines[label].set_data(self.time_data, self.wind_corrected_data[i])

        # Store environmental data
        self.pressure_data.append(pressure)
        self.humidity_data.append(humidity)
        avg_density = calculate_air_density(
            np.mean(rtd_temps), pressure, humidity
        )
        self.density_data.append(avg_density)

        # Dynamic x-axis adjustment
        time_window = 60
        if current_time > time_window:
            x_min = current_time - time_window
            x_max = current_time + 5
            self.ax.set_xlim(x_min, x_max)

        # Dynamic y-axis adjustment
        all_visible_data = []
        for label, line in self.lines.items():
            if line.get_visible() and len(line.get_xdata()) > 0:
                all_visible_data.extend(line.get_ydata())

        if all_visible_data:
            y_min = min(all_visible_data)
            y_max = max(all_visible_data)
            y_range = y_max - y_min
            if y_range < 2:
                y_range = 2
            y_min -= y_range * 0.1
            y_max += y_range * 0.1
            if y_min < 0: y_min = 0
            if y_max > 30: y_max = 30
            self.ax.set_ylim(y_min, y_max)

        # Update title
        if len(self.density_data) > 0:
            current_density = self.density_data[-1]
            current_pressure = self.pressure_data[-1]
            current_humidity = self.humidity_data[-1]
            self.fig.suptitle(
                f'RTD Wind Speed Monitoring - Interactive Combined View (Simulation) | Time: {current_time:.0f}s\n'
                f'Environment: Density={current_density:.3f} kg/m3, Pressure={current_pressure:.1f} kPa, Humidity={current_humidity:.1f}%RH',
                fontsize=12, fontweight='bold'
            )

        # Force a redraw
        self.fig.canvas.draw()

        return list(self.lines.values())

    def start(self):
        """Start real-time plotting"""
        print("\n" + "="*60)
        print("RTD Wind Speed Monitoring System - Interactive")
        print("="*60)
        print(f"Mode: {'Simulation' if self.simulation_mode else 'Real Device'}")
        print(f"Calibration: {self.CALIBRATION_TEMP}°C, {self.CALIBRATION_RH*100}%RH, {self.CALIBRATION_PRESSURE}kPa")
        print(f"Calibration density: {self.calibration_density:.3f} kg/m3")
        print("\nCheckbox Guide:")
        print("  - Raw: Original wind speed data")
        print("  - Filtered: Kalman filtered data")
        print("  - Corrected: Density corrected data")
        print("\nClick checkboxes to toggle line visibility")
        print("Press Ctrl+C to stop")
        print("="*60)

        # Enable interactive mode
        plt.ion()

        # Create animation
        self.ani = FuncAnimation(
            self.fig, self.update, interval=100,
            blit=False, repeat=True
        )

        # Show the plot
        plt.show(block=False)

        # Keep the script running
        try:
            while True:
                plt.pause(0.1)
        except KeyboardInterrupt:
            print("\n\nUser interrupted, stopping...")
        finally:
            plt.ioff()
            print("\nProgram stopped")

if __name__ == "__main__":
    # Create interactive plot in simulation mode
    plotter = RTDWindDensityInteractive(simulation_mode=True)
    plotter.start()