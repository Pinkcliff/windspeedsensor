"""
RTD Wind Speed Monitoring with Air Density Correction - Simulation Version
Uses simulated data to test matplotlib display without actual devices
"""

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import numpy as np
import time
import random
from Refrigerant import AIR
from kalman_filter import create_wind_speed_filter

def calculate_air_density(temperature: float, pressure: float, humidity: float) -> float:
    """计算空气密度"""
    try:
        air = AIR(dP=pressure, unit='c', dTdb=temperature, dRh=humidity)
        air.updateData()
        prop = air.getProp(unit='c')
        return prop['Density(kg/m3)']
    except:
        return 1.195  # 默认空气密度

class RTDWindDensitySimulator:
    def __init__(self):
        # 标定参数
        self.CALIBRATION_TEMP = 23.1  # °C
        self.CALIBRATION_RH = 0.65     # 65%
        self.CALIBRATION_PRESSURE = 101.325  # kPa
        self.calibration_density = calculate_air_density(
            self.CALIBRATION_TEMP,
            self.CALIBRATION_PRESSURE,
            self.CALIBRATION_RH * 100
        )

        # 数据存储
        self.max_points = 500
        self.time_data = deque()

        # 风速数据
        self.wind_raw_data = [deque() for _ in range(4)]
        self.wind_filtered_data = [deque() for _ in range(4)]
        self.wind_corrected_data = [deque() for _ in range(4)]

        # RTD温度数据
        self.rtd_temp_data = [deque() for _ in range(4)]

        # 环境数据
        self.pressure_data = deque()
        self.humidity_data = deque()
        self.density_data = deque()

        # 卡尔曼滤波器
        self.wind_filters = [create_wind_speed_filter() for _ in range(4)]

        # 模拟参数
        self.base_wind_speed = 5.0
        self.base_rtd_temp = [20.0, 21.0, 19.5, 20.5]
        self.base_pressure = 95.5
        self.base_humidity = 31.5

        # 设置matplotlib
        plt.style.use('default')
        self.fig, self.axes = plt.subplots(2, 2, figsize=(14, 10))
        self.fig.suptitle('RTD风速监测系统（模拟数据）\n'
                         f'标定条件: {self.CALIBRATION_TEMP}°C, {self.CALIBRATION_RH*100}%RH, {self.CALIBRATION_PRESSURE}kPa',
                         fontsize=14, fontweight='bold')

        plt.subplots_adjust(hspace=0.3, wspace=0.25)

        # 初始化绘图元素
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

            # 创建线条
            line_raw, = ax.plot([], [], color=colors[i], alpha=0.4, linewidth=1, label='原始值')
            line_filtered, = ax.plot([], [], color=colors[i], linewidth=2, label='滤波后')
            line_corrected, = ax.plot([], [], color='red', linewidth=2.5, label='密度修正后')

            self.lines_raw.append(line_raw)
            self.lines_filtered.append(line_filtered)
            self.lines_corrected.append(line_corrected)

            # 添加图例
            ax.legend(loc='upper right')

            # 当前值文本
            text_obj = ax.text(0.02, 0.95, '', transform=ax.transAxes,
                             verticalalignment='top', fontsize=9,
                             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
            self.current_value_texts.append(text_obj)

            # 密度文本
            density_obj = ax.text(0.98, 0.95, '', transform=ax.transAxes,
                                 verticalalignment='top', horizontalalignment='right', fontsize=9,
                                 bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.8))
            self.density_texts.append(density_obj)

            # 设置初始y轴范围
            ax.set_ylim(-2, 20)

        self.start_time = time.time()

    def generate_simulated_data(self):
        """生成模拟数据"""
        current_time = time.time() - self.start_time

        # 模拟风速变化
        wind_speeds_raw = []
        for i in range(4):
            # 基础风速 + 正弦变化 + 随机噪声
            wind_speed = self.base_wind_speed + \
                        2 * np.sin(current_time / 10 + i * np.pi/2) + \
                        random.gauss(0, 0.5)
            wind_speed = max(0, wind_speed)  # 确保非负
            wind_speeds_raw.append(wind_speed)

        # 模拟RTD温度变化
        rtd_temps = []
        for i in range(4):
            # 基础温度 + 缓慢变化 + 随机噪声
            temp = self.base_rtd_temp[i] + \
                  2 * np.sin(current_time / 30 + i) + \
                  random.gauss(0, 0.1)
            rtd_temps.append(temp)

        # 模拟压力和湿度变化
        pressure = self.base_pressure + random.gauss(0, 0.5)
        humidity = self.base_humidity + random.gauss(0, 1.0)

        return wind_speeds_raw, rtd_temps, pressure, humidity

    def update(self, frame):
        """更新数据和绘图"""
        current_time = time.time() - self.start_time

        # 生成模拟数据
        wind_speeds_raw, rtd_temps, pressure, humidity = self.generate_simulated_data()

        # 添加时间戳
        self.time_data.append(current_time)

        # 更新数据存储
        for i in range(4):
            # 存储原始风速
            self.wind_raw_data[i].append(wind_speeds_raw[i])

            # 应用卡尔曼滤波
            wind_speed_filtered = self.wind_filters[i].update(wind_speeds_raw[i])
            self.wind_filtered_data[i].append(wind_speed_filtered)

            # 存储RTD温度
            self.rtd_temp_data[i].append(rtd_temps[i])

            # 使用RTD温度计算空气密度
            density = calculate_air_density(rtd_temps[i], pressure, humidity)

            # 计算K因子和修正后的风速
            K = (self.calibration_density / density) ** 0.5 if density > 0 else 1.0
            wind_speed_corrected = wind_speed_filtered * K
            self.wind_corrected_data[i].append(wind_speed_corrected)

        # 存储环境数据
        self.pressure_data.append(pressure)
        self.humidity_data.append(humidity)
        avg_density = calculate_air_density(
            np.mean(rtd_temps), pressure, humidity
        )
        self.density_data.append(avg_density)

        # 更新绘图
        for i in range(4):
            # 更新线条数据
            self.lines_raw[i].set_data(self.time_data, self.wind_raw_data[i])
            self.lines_filtered[i].set_data(self.time_data, self.wind_filtered_data[i])
            self.lines_corrected[i].set_data(self.time_data, self.wind_corrected_data[i])

            # 动态调整x轴
            time_window = 60
            time_margin = 5

            if current_time > time_window * 0.8 and len(self.time_data) > 100:
                x_min = current_time - time_window
                x_max = current_time + time_margin
                self.axes.flat[i].set_xlim(x_min, x_max)
            else:
                self.axes.flat[i].set_xlim(0, max(time_window, current_time + time_margin))

            # 动态调整y轴
            if len(self.wind_raw_data[i]) > 0:
                recent_points = min(100, len(self.wind_raw_data[i]))
                recent_raw = list(self.wind_raw_data[i])[-recent_points:]
                recent_filtered = list(self.wind_filtered_data[i])[-recent_points:]
                recent_corrected = list(self.wind_corrected_data[i])[-recent_points:]

                if recent_raw and recent_filtered and recent_corrected:
                    all_data = recent_raw + recent_filtered + recent_corrected
                    y_min = min(all_data)
                    y_max = max(all_data)

                    # 添加边距
                    y_range = y_max - y_min
                    if y_range < 2:
                        y_range = 2
                    y_min -= y_range * 0.1
                    y_max += y_range * 0.1

                    # 确保最小值
                    if y_min < 0: y_min = 0
                    if y_max > 30: y_max = 30

                    self.axes.flat[i].set_ylim(y_min, y_max)

                    # 更新当前值文本
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

        # 更新标题
        if len(self.density_data) > 0:
            current_density = self.density_data[-1]
            current_pressure = self.pressure_data[-1]
            current_humidity = self.humidity_data[-1]

            self.fig.suptitle(
                f'RTD风速监测系统（模拟）| 运行时间: {current_time:.0f}s\n'
                f'当前环境: 密度={current_density:.3f} kg/m³, 压力={current_pressure:.1f} kPa, 湿度={current_humidity:.1f}%RH',
                fontsize=12, fontweight='bold'
            )

        return self.lines_raw + self.lines_filtered + self.lines_corrected

    def start(self):
        """启动实时绘图"""
        print("\n" + "="*60)
        print("RTD风速监测系统 - 模拟版本")
        print("="*60)
        print(f"标定条件: {self.CALIBRATION_TEMP}°C, {self.CALIBRATION_RH*100}%RH, {self.CALIBRATION_PRESSURE}kPa")
        print(f"标定密度: {self.calibration_density:.3f} kg/m³")
        print("显示内容: 原始值、卡尔曼滤波值、基于RTD温度的密度修正值")
        print("按Ctrl+C停止")
        print("="*60)

        # 创建动画
        self.ani = FuncAnimation(
            self.fig, self.update, interval=100,
            blit=False, cache_frame_data=False, repeat=True
        )

        try:
            plt.show()
        except KeyboardInterrupt:
            print("\n\n用户中断，停止程序...")
        finally:
            print("\n程序已停止")

if __name__ == "__main__":
    print("启动RTD风速监测模拟器...")
    plotter = RTDWindDensitySimulator()
    plotter.start()