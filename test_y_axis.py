"""
Test script to verify Y-axis dynamic scaling
Uses simulated data to test the functionality
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
from kalman_filter import create_wind_speed_filter
import random

class YAxisTestPlotter:
    def __init__(self):
        # Data storage
        self.max_points = 2000
        self.time_data = deque()
        self.wind_raw_data = [deque() for _ in range(4)]
        self.wind_filtered_data = [deque() for _ in range(4)]

        # Kalman filters
        self.wind_filters = [create_wind_speed_filter() for _ in range(4)]

        # Time
        self.start_time = 0
        self.frame_count = 0

        # Setup matplotlib
        self.fig, self.axes = plt.subplots(2, 2, figsize=(14, 10))
        self.fig.suptitle('Y-Axis Dynamic Scaling Test', fontsize=16, fontweight='bold')

        plt.subplots_adjust(hspace=0.3, wspace=0.25)

        # Initialize plots
        self.lines_raw = []
        self.lines_filtered = []
        self.current_value_texts = []
        colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728']

        for i, ax in enumerate(self.axes.flat):
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

            # Text for current values
            text_obj = ax.text(0.02, 0.95, '', transform=ax.transAxes,
                             verticalalignment='top', fontsize=10,
                             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
            self.current_value_texts.append(text_obj)

            # Initial y-axis range
            ax.set_ylim(0, 10)

            # Disable autoscale initially
            ax.set_autoscale_on(False)

    def generate_test_data(self, frame):
        """Generate test wind speed data with varying ranges"""
        t = frame * 0.1  # Time in seconds

        # Create different patterns for each sensor
        # Sensor 1: Sinusoidal variation from 0-15
        base1 = 7.5 + 7.5 * np.sin(t * 0.5)
        noise1 = random.gauss(0, 1)
        val1 = base1 + noise1

        # Sensor 2: Step changes every 100 frames
        if frame % 200 < 100:
            base2 = 5
        else:
            base2 = 20
        noise2 = random.gauss(0, 2)
        val2 = base2 + noise2

        # Sensor 3: Slowly increasing from 0-30
        base3 = min(30, frame * 0.15)
        noise3 = random.gauss(0, 1.5)
        val3 = base3 + noise3

        # Sensor 4: Random spikes
        base4 = 10
        if random.random() < 0.05:  # 5% chance of spike
            val4 = random.uniform(25, 30)
        else:
            noise4 = random.gauss(0, 2)
            val4 = base4 + noise4

        return [val1, val2, val3, val4]

    def update(self, frame):
        """Update function for animation"""
        self.frame_count = frame
        current_time = frame * 0.1

        # Generate test data
        raw_values = self.generate_test_data(frame)

        # Add timestamp
        self.time_data.append(current_time)

        # Process each sensor
        for i, raw_val in enumerate(raw_values):
            # Apply Kalman filter
            filtered_val = self.wind_filters[i].update(raw_val)

            # Store data
            self.wind_raw_data[i].append(raw_val)
            self.wind_filtered_data[i].append(filtered_val)

            # Update plots
            if len(self.time_data) > 1:
                # Update line data
                self.lines_raw[i].set_data(self.time_data, self.wind_raw_data[i])
                self.lines_filtered[i].set_data(self.time_data, self.wind_filtered_data[i])

                # Dynamic y-axis adjustment
                if len(self.wind_raw_data[i]) > 10:
                    # Get last 100 points or all if less
                    recent_points = min(100, len(self.wind_raw_data[i]))
                    recent_raw = list(self.wind_raw_data[i])[-recent_points:]
                    recent_filtered = list(self.wind_filtered_data[i])[-recent_points:]

                    all_data = recent_raw + recent_filtered
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
                    if y_max > 35: y_max = 35

                    # Force update - Multiple methods
                    ax = self.axes.flat[i]
                    ax.set_autoscale_on(False)  # Disable autoscale
                    ax.set_ylim(y_min, y_max)     # Set limits
                    ax.set_ybound(lower=y_min, upper=y_max)  # Alternative method

                    # Debug print every 50 frames
                    if frame % 50 == 0:
                        print(f"\nFrame {frame}: Sensor {i+1} Y-axis: [{y_min:.1f}, {y_max:.1f}], "
                              f"Current: Raw={raw_val:.2f}, Filtered={filtered_val:.2f}")

                # Update current value text
                self.current_value_texts[i].set_text(
                    f'Raw: {raw_val:.2f} m/s\n'
                    f'Filtered: {filtered_val:.2f} m/s\n'
                    f'Diff: {abs(raw_val - filtered_val):.2f}'
                )

        # Update x-axis to show last 30 seconds
        if current_time > 30:
            for ax in self.axes.flat:
                ax.set_xlim(current_time - 30, current_time)

        # Update title
        self.fig.suptitle(f'Y-Axis Dynamic Scaling Test | Frame: {frame} | Time: {current_time:.1f}s',
                         fontsize=14, fontweight='bold')

        return self.lines_raw + self.lines_filtered

    def start(self):
        """Start the animation"""
        print("\n" + "="*60)
        print("Y-Axis Dynamic Scaling Test")
        print("="*60)
        print("This test will generate varying wind speed data")
        print("Watch the Y-axis scale as data changes:")
        print("  - Sensor 1: Sinusoidal (0-15 m/s)")
        print("  - Sensor 2: Step changes (5-20 m/s)")
        print("  - Sensor 3: Gradual increase (0-30 m/s)")
        print("  - Sensor 4: Random spikes with base 10 m/s")
        print("\nPress Ctrl+C to stop")
        print("="*60)

        # Create animation
        ani = FuncAnimation(
            self.fig, self.update, interval=100,  # 100ms between frames
            blit=False, cache_frame_data=False
        )

        try:
            plt.show()
        except KeyboardInterrupt:
            print("\n\nTest stopped by user")

        print("\nTest completed!")


if __name__ == "__main__":
    test = YAxisTestPlotter()
    test.start()