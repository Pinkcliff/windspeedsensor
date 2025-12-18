import socket
import time
from typing import List, Dict, Optional
from Refrigerant import AIR
from kalman_filter import create_wind_speed_filter
import threading
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import numpy as np

# 简化的数据存储
class WindData:
    def __init__(self):
        # 使用列表代替deque，减少锁的使用
        self.max_points = 300
        self.time = []
        self.raw = [[], [], [], []]
        self.filtered = [[], [], [], []]
        self.corrected = [[], [], [], []]

    def update(self, raw_speeds, filtered_speeds, corrected_speeds):
        """更新数据 - 无锁版本"""
        current_time = time.time()

        # 更新时间
        self.time.append(current_time)
        if len(self.time) > self.max_points:
            self.time.pop(0)

        # 更新各路数据
        for i in range(4):
            self.raw[i].append(raw_speeds[i])
            self.filtered[i].append(filtered_speeds[i])
            self.corrected[i].append(corrected_speeds[i])

            if len(self.raw[i]) > self.max_points:
                self.raw[i].pop(0)
            if len(self.filtered[i]) > self.max_points:
                self.filtered[i].pop(0)
            if len(self.corrected[i]) > self.max_points:
                self.corrected[i].pop(0)


class SimplePlotter:
    def __init__(self, wind_data):
        self.wind_data = wind_data
        self.running = False

        # 创建图形
        plt.figure(figsize=(14, 10))
        self.axes = []
        self.lines_raw = []
        self.lines_filtered = []
        self.lines_corrected = []

        # 标定参数
        self.CALIBRATION_TEMP = 23.1
        self.CALIBRATION_RH = 0.65
        self.CALIBRATION_PRESSURE = 101.325
        self.calibration_density = self.calculate_air_density(
            self.CALIBRATION_TEMP,
            self.CALIBRATION_PRESSURE,
            self.CALIBRATION_RH * 100
        )

        # 创建子图
        titles = ['风速传感器 1 (RTD05)', '风速传感器 2 (RTD06)',
                 '风速传感器 3 (RTD07)', '风速传感器 4 (RTD08)']

        for i in range(4):
            ax = plt.subplot(2, 2, i+1)
            ax.set_title(titles[i])
            ax.set_xlabel('时间 (秒)')
            ax.set_ylabel('风速 (m/s)')
            ax.grid(True, alpha=0.3)
            ax.set_ylim(-2, 15)

            # 创建线条
            line_raw, = ax.plot([], [], 'r-', label='原始值', linewidth=1.5, alpha=0.7)
            line_filtered, = ax.plot([], [], 'b-', label='滤波后', linewidth=2)
            line_corrected, = ax.plot([], [], 'g-', label='修正后', linewidth=2)

            self.axes.append(ax)
            self.lines_raw.append(line_raw)
            self.lines_filtered.append(line_filtered)
            self.lines_corrected.append(line_corrected)
            ax.legend(loc='upper left')

        plt.tight_layout()
        plt.suptitle('风速实时监测', fontsize=16)

    def calculate_air_density(self, temperature, pressure, humidity):
        try:
            air = AIR(dP=pressure, unit='c', dTdb=temperature, dRh=humidity/100)
            air.updateData()
            prop = air.getProp(unit='c')
            return prop['Density(kg/m3)']
        except:
            return 0.0

    def update_plot(self, frame):
        """更新绘图"""
        if not self.wind_data.time:
            return self.lines_raw + self.lines_filtered + self.lines_corrected

        # 计算相对时间
        relative_time = [(t - self.wind_data.time[0]) for t in self.wind_data.time]

        # 更新每个子图
        for i in range(4):
            if len(relative_time) == len(self.wind_data.raw[i]):
                self.lines_raw[i].set_data(relative_time, self.wind_data.raw[i])
                self.lines_filtered[i].set_data(relative_time, self.wind_data.filtered[i])
                self.lines_corrected[i].set_data(relative_time, self.wind_data.corrected[i])

                # 调整坐标轴
                if relative_time:
                    self.axes[i].set_xlim(max(0, relative_time[-1] - 60), relative_time[-1] + 1)

                    all_data = (self.wind_data.raw[i][-20:] +
                               self.wind_data.filtered[i][-20:] +
                               self.wind_data.corrected[i][-20:])
                    if all_data:
                        y_min = min(all_data) - 0.5
                        y_max = max(all_data) + 0.5
                        self.axes[i].set_ylim(y_min, y_max)

        return self.lines_raw + self.lines_filtered + self.lines_corrected

    def run(self):
        """运行绘图"""
        self.running = True

        # 创建动画
        self.ani = animation.FuncAnimation(
            plt.gcf(), self.update_plot,
            interval=200,  # 200ms更新一次
            blit=False
        )

        plt.show()


# 模拟传感器数据
def simulate_sensor_data():
    """模拟传感器数据"""
    import random

    raw_speeds = [5.0, 4.8, 5.2, 4.9]
    filtered_speeds = [5.0, 4.8, 5.2, 4.9]
    rtd_temps = [16.7, 17.2, 16.5, 16.5]
    pressure = 95.5
    humidity = 31.5

    # 创建卡尔曼滤波器
    filters = [create_wind_speed_filter() for _ in range(4)]

    for _ in range(10000):
        # 模拟数据变化
        for i in range(4):
            # 原始数据
            raw_speeds[i] = max(0, 5 + random.gauss(0, 2))

            # 卡尔曼滤波
            filtered_speeds[i] = filters[i].update(raw_speeds[i])

            # RTD温度
            rtd_temps[i] += random.gauss(0, 0.05)
            rtd_temps[i] = max(10, min(30, rtd_temps[i]))

        # 压力和湿度
        pressure += random.gauss(0, 0.1)
        humidity += random.gauss(0, 0.2)
        pressure = max(90, min(110, pressure))
        humidity = max(20, min(80, humidity))

        # 计算修正值
        corrected_speeds = []
        for i in range(4):
            density = calculate_air_density(rtd_temps[i], pressure, humidity)
            calibration_density = 1.195
            K = (calibration_density / density) ** 0.5 if density > 0 else 1.0
            corrected_speed = filtered_speeds[i] * K
            corrected_speeds.append(corrected_speed)

        yield raw_speeds, filtered_speeds, corrected_speeds, rtd_temps, pressure, humidity

        time.sleep(0.1)


def calculate_air_density(temperature, pressure, humidity):
    """计算空气密度"""
    try:
        air = AIR(dP=pressure, unit='c', dTdb=temperature, dRh=humidity/100)
        air.updateData()
        prop = air.getProp(unit='c')
        return prop['Density(kg/m3)']
    except:
        return 0.0


def main():
    print("="*60)
    print("风速监测系统 - 无锁版本")
    print("="*60)

    # 创建数据存储
    wind_data = WindData()

    # 创建绘图器
    plotter = SimplePlotter(wind_data)

    # 在单独线程中运行绘图
    plotter_thread = threading.Thread(target=plotter.run)
    plotter_thread.daemon = True
    plotter_thread.start()

    # 等待窗口创建
    time.sleep(2)
    print("绘图窗口已启动！")
    print("数据更新中...\n")

    # 模拟数据
    try:
        for raw, filtered, corrected, rtd, p, h in simulate_sensor_data():
            wind_data.update(raw, filtered, corrected)

            # 每1秒打印一次状态
            if int(time.time()) % 10 == 0:
                print(f"压力: {p:.1f}kPa | 湿度: {h:.1f}%")

    except KeyboardInterrupt:
        print("\n程序已停止")

    plt.close('all')


if __name__ == "__main__":
    main()