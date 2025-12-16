"""
Wind Data Viewer
Read and visualize saved wind speed data from CSV files
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os
from datetime import datetime
import glob

class WindDataViewer:
    def __init__(self):
        self.data_dir = "wind_data"

    def list_data_files(self):
        """List all available data files"""
        if not os.path.exists(self.data_dir):
            print(f"No data directory found: {self.data_dir}")
            return []

        pattern = os.path.join(self.data_dir, "*_wind_data.csv")
        files = glob.glob(pattern)
        files.sort(reverse=True)  # Most recent first

        if not files:
            print("No data files found")
            return []

        print("\nAvailable data files:")
        print("-" * 80)
        for i, file in enumerate(files):
            filename = os.path.basename(file)
            timestamp_str = filename.replace('_wind_data.csv', '')
            try:
                # Parse timestamp
                dt = datetime.strptime(timestamp_str, "%Y%m%d_%H%M%S")
                formatted_time = dt.strftime("%Y-%m-%d %H:%M:%S")
                size = os.path.getsize(file) / 1024  # KB
                print(f"{i+1}. {filename} ({formatted_time}, {size:.1f} KB)")
            except:
                print(f"{i+1}. {filename}")

        return files

    def load_data(self, filename):
        """Load data from CSV file"""
        try:
            df = pd.read_csv(filename)
            print(f"\nLoaded {len(df)} data points from {filename}")
            return df
        except Exception as e:
            print(f"Error loading file: {e}")
            return None

    def plot_data(self, df, title=""):
        """Plot wind speed data"""
        if df is None or df.empty:
            print("No data to plot")
            return

        # Create figure with subplots
        fig, axes = plt.subplots(2, 2, figsize=(15, 10))
        if title:
            fig.suptitle(f'Wind Speed Data - {title}', fontsize=16, fontweight='bold')
        else:
            fig.suptitle('Wind Speed Data Analysis', fontsize=16, fontweight='bold')

        plt.subplots_adjust(hspace=0.3, wspace=0.25)

        colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728']

        for i in range(4):
            ax = axes.flat[i]
            sensor_num = i + 1

            # Plot raw data
            if not df[f'Wind{sensor_num}_Raw'].isna().all():
                ax.plot(df['Time_Elapsed_s'], df[f'Wind{sensor_num}_Raw'],
                       color=colors[i], alpha=0.5, linewidth=0.8, label='Raw')

            # Plot filtered data
            if not df[f'Wind{sensor_num}_Filtered'].isna().all():
                ax.plot(df['Time_Elapsed_s'], df[f'Wind{sensor_num}_Filtered'],
                       color=colors[i], linewidth=2, label='Filtered')

            # Calculate statistics
            raw_col = f'Wind{sensor_num}_Raw'
            filtered_col = f'Wind{sensor_num}_Filtered'
            if not df[raw_col].isna().all():
                raw_std = df[raw_col].std()
                raw_mean = df[raw_col].mean()
            else:
                raw_std = raw_mean = 0

            if not df[filtered_col].isna().all():
                filtered_std = df[filtered_col].std()
                filtered_mean = df[filtered_col].mean()
                noise_reduction = (raw_std - filtered_std) / raw_std * 100 if raw_std > 0 else 0
            else:
                filtered_std = filtered_mean = 0
                noise_reduction = 0

            # Set title and labels
            ax.set_title(f'Wind Speed Sensor {sensor_num}', fontsize=12)
            ax.set_xlabel('Time (seconds)', fontsize=10)
            ax.set_ylabel('Wind Speed (m/s)', fontsize=10)
            ax.grid(True, alpha=0.3)
            ax.legend(loc='upper right')

            # Add statistics text
            stats_text = f'Mean: {filtered_mean:.2f} m/s\n'
            stats_text += f'Std Dev: {filtered_std:.3f} m/s\n'
            stats_text += f'Noise Reduction: {noise_reduction:.1f}%'
            ax.text(0.02, 0.98, stats_text, transform=ax.transAxes,
                   verticalalignment='top',
                   bbox=dict(boxstyle='round', facecolor='white', alpha=0.8),
                   fontsize=9)

        plt.tight_layout()
        return fig

    def plot_summary(self, df):
        """Plot summary statistics"""
        if df is None or df.empty:
            print("No data to analyze")
            return

        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 10))
        fig.suptitle('Wind Speed Data Summary', fontsize=16, fontweight='bold')

        # 1. Average wind speeds
        sensors = ['Sensor 1', 'Sensor 2', 'Sensor 3', 'Sensor 4']
        raw_means = []
        filtered_means = []
        for i in range(1, 5):
            raw_means.append(df[f'Wind{i}_Raw'].mean())
            filtered_means.append(df[f'Wind{i}_Filtered'].mean())

        x = np.arange(len(sensors))
        width = 0.35
        ax1.bar(x - width/2, raw_means, width, label='Raw Mean', alpha=0.8)
        ax1.bar(x + width/2, filtered_means, width, label='Filtered Mean', alpha=0.8)
        ax1.set_xlabel('Sensor')
        ax1.set_ylabel('Wind Speed (m/s)')
        ax1.set_title('Average Wind Speeds')
        ax1.set_xticks(x)
        ax1.set_xticklabels(sensors)
        ax1.legend()
        ax1.grid(True, alpha=0.3)

        # 2. Standard deviations
        raw_stds = []
        filtered_stds = []
        for i in range(1, 5):
            raw_stds.append(df[f'Wind{i}_Raw'].std())
            filtered_stds.append(df[f'Wind{i}_Filtered'].std())

        ax2.bar(x - width/2, raw_stds, width, label='Raw Std Dev', alpha=0.8, color='orange')
        ax2.bar(x + width/2, filtered_stds, width, label='Filtered Std Dev', alpha=0.8, color='red')
        ax2.set_xlabel('Sensor')
        ax2.set_ylabel('Standard Deviation (m/s)')
        ax2.set_title('Data Variability (Lower is better)')
        ax2.set_xticks(x)
        ax2.set_xticklabels(sensors)
        ax2.legend()
        ax2.grid(True, alpha=0.3)

        # Add noise reduction percentages
        for i in range(len(sensors)):
            if raw_stds[i] > 0:
                reduction = (raw_stds[i] - filtered_stds[i]) / raw_stds[i] * 100
                ax2.text(i, filtered_stds[i] + 0.1, f'{reduction:.1f}%',
                        ha='center', va='bottom', fontweight='bold')

        # 3. Temperature and Pressure (if available)
        if 'Temperature' in df.columns and not df['Temperature'].isna().all():
            ax3.plot(df['Time_Elapsed_s'], df['Temperature'], 'r-', label='Temperature')
            ax3.set_xlabel('Time (seconds)')
            ax3.set_ylabel('Temperature (°C)', color='r')
            ax3.tick_params(axis='y', labelcolor='r')
            ax3.grid(True, alpha=0.3)
            ax3.set_title('Temperature')
        else:
            ax3.text(0.5, 0.5, 'No Temperature Data', ha='center', va='center', transform=ax3.transAxes)
            ax3.set_title('Temperature (Not Available)')

        # 4. Success rate over time
        if 'Success_Flag' in df.columns:
            # Calculate rolling success rate
            window = min(100, len(df) // 10)
            if window > 0:
                df['Success_Rate'] = df['Success_Flag'].rolling(window=window, min_periods=1).mean() * 100
                ax4.plot(df['Time_Elapsed_s'], df['Success_Rate'], 'g-')
                ax4.set_xlabel('Time (seconds)')
                ax4.set_ylabel('Success Rate (%)')
                ax4.set_ylim(0, 105)
                ax4.grid(True, alpha=0.3)
                ax4.set_title(f'Data Acquisition Success Rate (Rolling {window} points)')
            else:
                ax4.text(0.5, 0.5, 'Insufficient Data', ha='center', va='center', transform=ax4.transAxes)

        plt.tight_layout()
        return fig

    def run(self):
        """Run the viewer"""
        print("="*60)
        print("Wind Speed Data Viewer")
        print("="*60)

        # List available files
        files = self.list_data_files()
        if not files:
            return

        # Select file
        while True:
            try:
                choice = input("\nEnter file number (or 'q' to quit): ")
                if choice.lower() == 'q':
                    return

                file_idx = int(choice) - 1
                if 0 <= file_idx < len(files):
                    selected_file = files[file_idx]
                    break
                else:
                    print("Invalid selection. Please try again.")
            except ValueError:
                print("Invalid input. Please enter a number.")

        # Load and display data
        filename = os.path.basename(selected_file)
        df = self.load_data(selected_file)

        if df is not None:
            # Display basic statistics
            print("\nData Statistics:")
            print("-" * 60)
            print(f"Duration: {df['Time_Elapsed_s'].max():.1f} seconds")
            print(f"Total readings: {len(df)}")

            if 'Success_Flag' in df.columns:
                success_rate = df['Success_Flag'].mean() * 100
                print(f"Success rate: {success_rate:.1f}%")

            print("\nWind Speed Statistics (Filtered):")
            for i in range(1, 5):
                col = f'Wind{i}_Filtered'
                if not df[col].isna().all():
                    mean_val = df[col].mean()
                    std_val = df[col].std()
                    max_val = df[col].max()
                    min_val = df[col].min()
                    print(f"  Sensor {i}: Mean={mean_val:.2f}, Std={std_val:.3f}, "
                          f"Min={min_val:.2f}, Max={max_val:.2f} m/s")

            # Plot data
            print("\nGenerating plots...")
            fig1 = self.plot_data(df, filename)
            fig2 = self.plot_summary(df)

            # Save plots
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            plot1_name = f"wind_data_{timestamp}_detail.png"
            plot2_name = f"wind_data_{timestamp}_summary.png"
            fig1.savefig(plot1_name, dpi=300, bbox_inches='tight')
            fig2.savefig(plot2_name, dpi=300, bbox_inches='tight')
            print(f"\nPlots saved as:")
            print(f"  - {plot1_name}")
            print(f"  - {plot2_name}")

            plt.show()


if __name__ == "__main__":
    viewer = WindDataViewer()
    viewer.run()