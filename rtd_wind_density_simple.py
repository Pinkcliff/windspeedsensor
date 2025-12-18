import socket
import time
from typing import List, Dict, Optional
from Refrigerant import AIR
from kalman_filter import create_wind_speed_filter
import threading
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import numpy as np

# 复制核心函数（省略重复代码，直接使用之前的实现）
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

def parse_rtu_response(response_bytes: bytes) -> Dict:
    response = list(response_bytes)
    if len(response) < 4:
        return {"error": "响应帧过短"}

    slave_addr = response[0]
    func_code = response[1]
    data = response[2:-2]
    received_crc = response[-2:]

    calculated_crc = modbus_crc(response[:-2])
    if received_crc != calculated_crc:
        return {"error": f"CRC校验失败"}

    if func_code in [0x03, 0x04]:
        if len(data) < 1:
            return {"error": f"功能码{func_code:02X}响应数据为空"}
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
        return {"error": f"不支持的功能码：0x{func_code:02X}"}


# 简化的数据收集器
class DataCollector:
    def __init__(self):
        # 标定参数
        self.CALIBRATION_TEMP = 23.1
        self.CALIBRATION_RH = 0.65
        self.CALIBRATION_PRESSURE = 101.325
        self.calibration_density = self.calculate_air_density(
            self.CALIBRATION_TEMP,
            self.CALIBRATION_PRESSURE,
            self.CALIBRATION_RH * 100
        )

        # 数据存储
        self.max_points = 300
        self.time_data = []
        self.wind_raw = [[], [], [], []]
        self.wind_filtered = [[], [], [], []]
        self.wind_corrected = [[], [], [], []]

        # 线条对象
        self.lines_raw = []
        self.lines_filtered = []
        self.lines_corrected = []

        # 创建图形
        self.setup_plot()

    def calculate_air_density(self, temperature, pressure, humidity):
        try:
            air = AIR(dP=pressure, unit='c', dTdb=temperature, dRh=humidity/100)
            air.updateData()
            prop = air.getProp(unit='c')
            return prop['Density(kg/m3)']
        except:
            return 0.0

    def setup_plot(self):
        # 创建图形
        plt.figure(figsize=(14, 10))
        plt.suptitle('风速实时监测 - 原始值/滤波值/修正后\n'
                    f'标定: T={self.CALIBRATION_TEMP}℃, RH={self.CALIBRATION_RH*100:.0f}%, P={self.CALIBRATION_PRESSURE:.1f}kPa',
                    fontsize=14)

        # 创建4个子图
        for i in range(4):
            plt.subplot(2, 2, i+1)
            plt.title(f'风速传感器 {i+1} (RTD{5+i})', fontsize=12)

            # 创建线条
            line_raw, = plt.plot([], [], 'r-', label='原始值', linewidth=1, alpha=0.7)
            line_filtered, = plt.plot([], [], 'b-', label='滤波后', linewidth=2)
            line_corrected, = plt.plot([], [], 'g-', label='修正后', linewidth=2)

            self.lines_raw.append(line_raw)
            self.lines_filtered.append(line_filtered)
            self.lines_corrected.append(line_corrected)

            plt.xlabel('时间 (秒)')
            plt.ylabel('风速 (m/s)')
            plt.grid(True, alpha=0.3)
            plt.legend(loc='upper left')
            plt.ylim(-2, 15)

        plt.tight_layout()

    def update_data(self, raw_speeds, filtered_speeds, rtd_temps, pressure, humidity):
        """更新数据"""
        current_time = time.time()

        # 更新时间轴
        self.time_data.append(current_time)
        if len(self.time_data) > self.max_points:
            self.time_data.pop(0)

        # 计算相对时间
        if self.time_data:
            relative_time = [(t - self.time_data[0]) for t in self.time_data]
        else:
            relative_time = []

        # 计算修正后的风速
        corrected_speeds = []
        for i in range(4):
            density = self.calculate_air_density(rtd_temps[i], pressure, humidity)
            K = (self.calibration_density / density) ** 0.5 if density > 0 else 1.0
            corrected_speed = filtered_speeds[i] * K
            corrected_speeds.append(corrected_speed)

            # 更新数据
            self.wind_raw[i].append(raw_speeds[i])
            self.wind_filtered[i].append(filtered_speeds[i])
            self.wind_corrected[i].append(corrected_speed)

            if len(self.wind_raw[i]) > self.max_points:
                self.wind_raw[i].pop(0)
            if len(self.wind_filtered[i]) > self.max_points:
                self.wind_filtered[i].pop(0)
            if len(self.wind_corrected[i]) > self.max_points:
                self.wind_corrected[i].pop(0)

            # 更新线条
            if len(relative_time) == len(self.wind_raw[i]):
                self.lines_raw[i].set_data(relative_time, self.wind_raw[i])
                self.lines_filtered[i].set_data(relative_time, self.wind_filtered[i])
                self.lines_corrected[i].set_data(relative_time, self.wind_corrected[i])

                # 调整坐标轴
                if relative_time:
                    plt.subplot(2, 2, i+1)
                    plt.xlim(max(0, relative_time[-1] - 60), relative_time[-1] + 1)

                    all_data = self.wind_raw[i] + self.wind_filtered[i] + self.wind_corrected[i]
                    if all_data:
                        y_min = min(all_data) - 0.5
                        y_max = max(all_data) + 0.5
                        plt.ylim(y_min, y_max)

                    # 更新标题显示最新值
                    plt.title(f'风速传感器 {i+1}\n'
                            f'原始={raw_speeds[i]:.2f} | 滤波={filtered_speeds[i]:.2f} | 修正={corrected_speed:.2f}',
                            fontsize=10)

        # 更新主标题
        plt.suptitle(f'风速实时监测 - 原始值/滤波值/修正后 | '
                    f'时间: {time.strftime("%H:%M:%S")} | '
                    f'压力: {pressure:.1f}kPa | 湿度: {humidity:.1f}%\n'
                    f'标定: T={self.CALIBRATION_TEMP}℃, RH={self.CALIBRATION_RH*100:.0f}%, '
                    f'P={self.CALIBRATION_PRESSURE:.1f}kPa (密度={self.calibration_density:.3f}kg/m³)',
                    fontsize=12)

        plt.draw()
        plt.pause(0.001)


# 简单的传感器连接类（模拟数据）
class SimpleSensorReader:
    def __init__(self, data_collector):
        self.data_collector = data_collector
        self.running = False

        # 模拟数据初始化
        self.raw_speeds = [5.0, 4.8, 5.2, 4.9]
        self.filtered_speeds = [5.0, 4.8, 5.2, 4.9]
        self.rtd_temps = [16.7, 17.2, 16.5, 16.5]
        self.pressure = 95.5
        self.humidity = 31.5

        # 卡尔曼滤波器
        self.filters = [create_wind_speed_filter() for _ in range(4)]

    def run(self):
        """运行传感器读取（模拟）"""
        self.running = True
        print("传感器数据模拟已启动...")

        import random

        while self.running:
            # 模拟数据变化
            for i in range(4):
                # 原始数据添加噪声
                noise = random.gauss(0, 0.5)
                self.raw_speeds[i] = max(0, 5 + noise + random.gauss(0, 1))

                # 应用卡尔曼滤波
                self.filtered_speeds[i] = self.filters[i].update(self.raw_speeds[i])

                # RTD温度缓慢变化
                self.rtd_temps[i] += random.gauss(0, 0.05)
                self.rtd_temps[i] = max(10, min(30, self.rtd_temps[i]))

            # 压力和湿度缓慢变化
            self.pressure += random.gauss(0, 0.1)
            self.humidity += random.gauss(0, 0.2)
            self.pressure = max(90, min(110, self.pressure))
            self.humidity = max(20, min(80, self.humidity))

            # 更新图形
            self.data_collector.update_data(
                self.raw_speeds,
                self.filtered_speeds,
                self.rtd_temps,
                self.pressure,
                self.humidity
            )

            # 等待
            time.sleep(0.1)


def main():
    print("="*60)
    print("风速监测系统 - 测试版本")
    print("="*60)
    print("\n正在启动...")
    print("如果看不到窗口，请：")
    print("1. 检查任务栏")
    print("2. 按Alt+Tab")
    print("3. 查看是否有matplotlib窗口")

    # 创建数据收集器
    collector = DataCollector()

    # 创建并启动传感器
    sensor = SimpleSensorReader(collector)

    # 显示窗口
    plt.show(block=False)

    # 在单独的线程中运行传感器
    import threading
    sensor_thread = threading.Thread(target=sensor.run)
    sensor_thread.start()

    try:
        # 保持程序运行
        while True:
            time.sleep(1)
            # 每10秒打印一次状态
            if int(time.time()) % 10 == 0:
                print(f"运行中... 最新压力: {sensor.pressure:.1f}kPa")

    except KeyboardInterrupt:
        print("\n正在停止...")
        sensor.running = False
        sensor_thread.join()
        plt.close('all')
        print("程序已退出")


if __name__ == "__main__":
    main()