import socket
import time
from typing import List, Dict, Optional
from Refrigerant import AIR
from kalman_filter import create_wind_speed_filter
import threading
from collections import deque
import numpy as np
import os
import sys

# 复制之前的代码，但移除matplotlib相关
# （这里只是为了演示思路，实际使用时应该复制完整代码）

class ASCIIPlotter:
    """ASCII艺术风格的实时绘图"""
    def __init__(self, shared_data, width=80, height=10):
        self.shared_data = shared_data
        self.width = width
        self.height = height
        self.running = False

    def clear_screen(self):
        """清屏"""
        os.system('cls' if os.name == 'nt' else 'clear')

    def draw_plot(self, data, title, min_val=-5, max_val=20):
        """用ASCII字符绘制简单的波形图"""
        print(f"\n{title}")
        print("-" * self.width)

        # 数据归一化
        normalized = []
        for d in data:
            if max_val > min_val:
                n = int((d - min_val) / (max_val - min_val) * (self.height - 1))
                n = max(0, min(self.height - 1, n))
            else:
                n = self.height // 2
            normalized.append(n)

        # 从上到下绘制
        for row in range(self.height - 1, -1, -1):
            line = "|"
            for col in range(min(self.width, len(data))):
                if normalized[col] == row:
                    line += "*"
                elif normalized[col] > row:
                    line += "|"
                else:
                    line += " "
            line += "|"
            print(line)

        print("-" * self.width)

        # 显示最新值
        if data:
            print(f"最新值: 原始={data[-1][0]:.2f} | 滤波={data[-1][1]:.2f} | 修正={data[-1][2]:.2f}")

    def run(self):
        """运行ASCII绘图"""
        self.running = True
        self.clear_screen()

        while self.running:
            # 获取最新数据
            with self.shared_data.lock:
                data_for_plot = []
                for i in range(4):
                    # 获取最近的数据点
                    raw = list(self.shared_data.wind_raw_history[i])[-self.width:]
                    filtered = list(self.shared_data.wind_filtered_history[i])[-self.width:]
                    corrected = list(self.shared_data.wind_corrected_history[i])[-self.width:]

                    # 组合数据
                    combined = [(r, f, c) for r, f, c in zip(raw, filtered, corrected)]
                    data_for_plot.append(combined)

            # 清屏并绘制
            self.clear_screen()
            print("=" * 90)
            print("风速实时监测 - ASCII图形模式")
            print("=" * 90)

            titles = ['风速传感器 1 (RTD05)', '风速传感器 2 (RTD06)',
                     '风速传感器 3 (RTD07)', '风速传感器 4 (RTD08)']

            for i in range(4):
                if data_for_plot[i]:
                    self.draw_plot(data_for_plot[i], titles[i])
                print()

            print("\n按 Ctrl+C 退出")

            # 等待
            for _ in range(10):
                if not self.running:
                    break
                time.sleep(0.1)

if __name__ == "__main__":
    # 测试ASCII绘图
    import random

    # 模拟数据
    data = [(random.uniform(0, 10), random.uniform(0, 10), random.uniform(0, 10))
            for _ in range(60)]

    plotter = ASCIIPlotter(None, width=60)
    plotter.draw_plot(data, "测试 - 风速传感器 1", 0, 12)