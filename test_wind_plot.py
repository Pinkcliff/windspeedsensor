"""
Wind Speed Data Plotting Test
Generate simulated data and demonstrate Kalman filter effects
"""

import numpy as np
import matplotlib.pyplot as plt
from kalman_filter import create_wind_speed_filter
import random
from datetime import datetime, timedelta

def simulate_wind_data(n_points=200, duration=60):
    """
    Simulate wind speed data

    Parameters:
        n_points: Number of data points
        duration: Duration in seconds
    """
    # Generate time axis
    time_points = np.linspace(0, duration, n_points)

    # Generate simulated data for 4 sensors
    wind_data = []

    # Sensor 1: Stable wind speed with noise
    base_wind = 5.0
    noise1 = np.random.normal(0, 0.5, n_points)
    wind1 = base_wind + noise1

    # Sensor 2: Gust pattern
    wind2 = []
    for t in time_points:
        # Base wind + gust + noise
        gust = 3 * np.sin(t/5) * (np.random.random() > 0.7)  # 30% chance of gust
        noise = np.random.normal(0, 0.8)
        wind2.append(8 + gust + noise)
    wind2 = np.array(wind2)

    # Sensor 3: Slowly varying wind speed
    wind3 = 6 + 3 * np.sin(time_points/10) + np.random.normal(0, 0.6, n_points)

    # Sensor 4: Sudden changes
    wind4 = []
    current_wind = 4
    for t in time_points:
        if random.random() < 0.02:  # 2% chance of sudden change
            current_wind = random.uniform(2, 12)
        noise = np.random.normal(0, 0.4)
        wind4.append(current_wind + noise)
    wind4 = np.array(wind4)

    return time_points, [wind1, wind2, wind3, wind4]

def apply_kalman_filter(wind_data):
    """Apply Kalman filter to wind speed data"""
    filtered_data = []

    for i, sensor_data in enumerate(wind_data):
        # Create filter
        kalman_filter = create_wind_speed_filter()

        # Apply filter
        filtered = []
        for value in sensor_data:
            filtered_value = kalman_filter.update(value)
            filtered.append(filtered_value)

        filtered_data.append(np.array(filtered))

    return filtered_data

def plot_wind_data(time_points, wind_data, filtered_data):
    """Plot wind speed data"""
    # Create figure
    fig, axes = plt.subplots(2, 2, figsize=(15, 10))
    fig.suptitle('Wind Speed Sensor Data Comparison (Raw vs Kalman Filtered)',
                 fontsize=16, fontweight='bold')

    colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728']
    sensor_labels = ['Sensor 1 (Stable)', 'Sensor 2 (Gusty)',
                     'Sensor 3 (Gradual)', 'Sensor 4 (Sudden)']

    for idx, (ax, raw_data, filtered_data, color, label) in enumerate(
        zip(axes.flat, wind_data, filtered_data, colors, sensor_labels)):

        # Plot raw data
        ax.plot(time_points, raw_data, color=color, alpha=0.5,
                linewidth=0.8, label='Raw')

        # Plot filtered data
        ax.plot(time_points, filtered_data, color=color,
                linewidth=2, label='Filtered')

        # Set title and labels
        ax.set_title(f'{label}', fontsize=12, fontweight='bold')
        ax.set_xlabel('Time (seconds)', fontsize=10)
        ax.set_ylabel('Wind Speed (m/s)', fontsize=10)
        ax.grid(True, alpha=0.3)
        ax.legend(loc='upper right')

        # Calculate statistics
        raw_std = np.std(raw_data)
        filtered_std = np.std(filtered_data)
        noise_reduction = (raw_std - filtered_std) / raw_std * 100

        # Display statistics on plot
        stats_text = f'Raw Std Dev: {raw_std:.3f} m/s\n'
        stats_text += f'Filtered Std Dev: {filtered_std:.3f} m/s\n'
        stats_text += f'Noise Reduction: {noise_reduction:.1f}%'
        ax.text(0.02, 0.98, stats_text, transform=ax.transAxes,
                verticalalignment='top',
                bbox=dict(boxstyle='round', facecolor='white', alpha=0.8),
                fontsize=9)

    # Adjust layout
    plt.tight_layout()
    plt.subplots_adjust(top=0.93)

    # Save figure
    plt.savefig('wind_speed_comparison.png', dpi=300, bbox_inches='tight')
    print("Chart saved as 'wind_speed_comparison.png'")

    plt.show()

def plot_summary_statistics(time_points, wind_data, filtered_data):
    """Plot summary statistics"""
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))

    # Calculate means
    raw_avg = [np.mean(data) for data in wind_data]
    filtered_avg = [np.mean(data) for data in filtered_data]

    # Calculate standard deviations
    raw_std = [np.std(data) for data in wind_data]
    filtered_std = [np.std(data) for data in filtered_data]

    sensor_labels = ['Sensor 1', 'Sensor 2', 'Sensor 3', 'Sensor 4']
    x = np.arange(len(sensor_labels))
    width = 0.35

    # Mean comparison
    ax1.bar(x - width/2, raw_avg, width, label='Raw Mean', alpha=0.8)
    ax1.bar(x + width/2, filtered_avg, width, label='Filtered Mean', alpha=0.8)
    ax1.set_xlabel('Sensor')
    ax1.set_ylabel('Average Wind Speed (m/s)')
    ax1.set_title('Average Wind Speed Comparison')
    ax1.set_xticks(x)
    ax1.set_xticklabels(sensor_labels)
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    # Standard deviation comparison
    ax2.bar(x - width/2, raw_std, width, label='Raw Std Dev',
            alpha=0.8, color='orange')
    ax2.bar(x + width/2, filtered_std, width, label='Filtered Std Dev',
            alpha=0.8, color='red')
    ax2.set_xlabel('Sensor')
    ax2.set_ylabel('Standard Deviation (m/s)')
    ax2.set_title('Data Stability Comparison (Lower std dev = more stable)')
    ax2.set_xticks(x)
    ax2.set_xticklabels(sensor_labels)
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    # Add noise reduction percentage
    for i in range(len(sensor_labels)):
        reduction = (raw_std[i] - filtered_std[i]) / raw_std[i] * 100
        ax2.text(i, filtered_std[i] + 0.1, f'{reduction:.1f}%',
                ha='center', va='bottom', fontweight='bold')

    plt.tight_layout()
    plt.savefig('wind_speed_statistics.png', dpi=300, bbox_inches='tight')
    print("Statistics chart saved as 'wind_speed_statistics.png'")
    plt.show()

def main():
    """Main function"""
    print("="*60)
    print("Wind Speed Data Filtering Effect Test")
    print("="*60)

    # Generate simulated data
    print("Generating simulated wind speed data...")
    time_points, wind_data = simulate_wind_data(n_points=500, duration=120)

    # Apply Kalman filter
    print("Applying Kalman filter...")
    filtered_data = apply_kalman_filter(wind_data)

    # Print statistics
    print("\nStatistics:")
    print("-"*60)
    for i in range(4):
        raw_std = np.std(wind_data[i])
        filtered_std = np.std(filtered_data[i])
        noise_reduction = (raw_std - filtered_std) / raw_std * 100

        print(f"Sensor {i+1}:")
        print(f"  Raw data std dev: {raw_std:.4f} m/s")
        print(f"  Filtered data std dev: {filtered_std:.4f} m/s")
        print(f"  Noise reduction: {noise_reduction:.2f}%")
        print()

    # Plot comparison charts
    print("Plotting comparison charts...")
    plot_wind_data(time_points, wind_data, filtered_data)

    # Plot statistics charts
    print("Plotting statistics charts...")
    plot_summary_statistics(time_points, wind_data, filtered_data)

    print("\nTest completed!")

if __name__ == "__main__":
    main()