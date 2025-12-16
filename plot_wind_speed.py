"""
实时绘制风速数据
显示四个风速传感器的原始值和滤波后的值
"""

import socket
import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
from kalman_filter import create_wind_speed_filter
from typing import List, Optional

# Modbus RTU 帧处理函数
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


class WindSpeedPlotter:
    def __init__(self):
        # 设备参数
        self.DEVICE_IP = "192.168.0.101"
        self.DEVICE_PORT = 8234
        self.SLAVE_ADDR = 1
        self.FUNC_CODE = 0x04
        self.START_REG = 0
        self.REG_COUNT = 8
        self.TIMEOUT = 5
        self.BUFFER_SIZE = 1024

        # 数据存储 - 使用deque固定大小，自动删除旧数据
        self.max_points = 100  # 显示最近100个数据点
        self.time_data = deque(maxlen=self.max_points)

        # 原始风速数据
        self.wind_raw_data = [
            deque(maxlen=self.max_points),  # 风速1
            deque(maxlen=self.max_points),  # 风速2
            deque(maxlen=self.max_points),  # 风速3
            deque(maxlen=self.max_points)   # 风速4
        ]

        # 滤波后风速数据
        self.wind_filtered_data = [
            deque(maxlen=self.max_points),  # 风速1
            deque(maxlen=self.max_points),  # 风速2
            deque(maxlen=self.max_points),  # 风速3
            deque(maxlen=self.max_points)   # 风速4
        ]

        # 卡尔曼滤波器
        self.wind_filters = [create_wind_speed_filter() for _ in range(4)]

        # 连接对象
        self.sock: Optional[socket.socket] = None
        self.connected = False

        # 统计信息
        self.read_count = 0
        self.success_count = 0
        self.fail_count = 0
        self.start_time = time.time()

        # 设置matplotlib
        plt.style.use('seaborn-v0_8-darkgrid')
        self.fig, self.axes = plt.subplots(2, 2, figsize=(14, 10))
        self.fig.suptitle('风速实时监测（原始值 vs 卡尔曼滤波值）', fontsize=16, fontweight='bold')

        # 调整子图间距
        plt.subplots_adjust(hspace=0.3, wspace=0.25)

        # 初始化4个子图
        self.lines_raw = []
        self.lines_filtered = []
        colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728']

        for i, ax in enumerate(self.axes.flat):
            # 设置标题和标签
            ax.set_title(f'风速传感器 {i+1}', fontsize=12)
            ax.set_xlabel('时间 (秒)', fontsize=10)
            ax.set_ylabel('风速 (m/s)', fontsize=10)
            ax.grid(True, alpha=0.3)

            # 创建线条对象
            line_raw, = ax.plot([], [], color=colors[i], alpha=0.5, linewidth=1, label='原始值')
            line_filtered, = ax.plot([], [], color=colors[i], linewidth=2, label='滤波值')

            self.lines_raw.append(line_raw)
            self.lines_filtered.append(line_filtered)

            # 添加图例
            ax.legend(loc='upper right')

            # 设置y轴范围
            ax.set_ylim(-1, 20)

        # 启动连接
        self.connect_device()

    def connect_device(self) -> bool:
        """连接设备"""
        try:
            if self.sock:
                self.sock.close()

            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(self.TIMEOUT)
            self.sock.connect((self.DEVICE_IP, self.DEVICE_PORT))
            self.connected = True
            print(f"✅ 成功连接到设备 {self.DEVICE_IP}:{self.DEVICE_PORT}")
            return True
        except Exception as e:
            print(f"❌ 连接失败: {str(e)}")
            self.connected = False
            return False

    def read_wind_data(self):
        """读取风速数据"""
        if not self.connected or not self.sock:
            return None

        try:
            # 构建请求
            request = build_rtu_request(
                slave_addr=self.SLAVE_ADDR,
                start_reg=self.START_REG,
                reg_count=self.REG_COUNT,
                func_code=self.FUNC_CODE
            )

            # 发送请求
            self.sock.sendall(request)

            # 接收响应
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
                    raise socket.timeout(f"接收超时")
                time.sleep(0.01)

            # 解析响应
            parsed_data = parse_rtu_response(response_bytes)
            if "error" in parsed_data:
                print(f"⚠️ 解析失败: {parsed_data['error']}")
                return None

            registers = parsed_data["registers"]
            if len(registers) < self.REG_COUNT:
                print(f"⚠️ 数据不足：需要{self.REG_COUNT}个，收到{len(registers)}个")
                return None

            # 提取风速数据（寄存器4-7）
            wind_speeds = []
            for i in range(4, 8):
                raw_value = registers[i]
                current_value = raw_value / 249  # 转换为电流值(mA)
                wind_speed_raw = (current_value - 4) * 30 / 16  # 转换为风速(m/s)

                # 应用卡尔曼滤波
                wind_speed_filtered = self.wind_filters[i-4].update(wind_speed_raw)

                wind_speeds.append((wind_speed_raw, wind_speed_filtered))

            return wind_speeds

        except Exception as e:
            print(f"❌ 读取失败: {str(e)}")
            # 尝试重连
            if "Connection" in str(e) or "reset" in str(e):
                self.connected = False
                self.connect_device()
            return None

    def update(self, frame):
        """更新数据"""
        current_time = time.time() - self.start_time

        # 读取数据
        wind_data = self.read_wind_data()

        if wind_data:
            self.read_count += 1
            self.success_count += 1

            # 添加时间戳
            self.time_data.append(current_time)

            # 更新数据
            for i, (raw_val, filtered_val) in enumerate(wind_data):
                self.wind_raw_data[i].append(raw_val)
                self.wind_filtered_data[i].append(filtered_val)

            # 更新图表
            for i in range(4):
                if len(self.time_data) > 0:
                    # 更新原始数据线
                    self.lines_raw[i].set_data(self.time_data, self.wind_raw_data[i])
                    # 更新滤波数据线
                    self.lines_filtered[i].set_data(self.time_data, self.wind_filtered_data[i])

                    # 动态调整x轴范围
                    if current_time > 30:  # 如果超过30秒，只显示最近30秒
                        self.axes.flat[i].set_xlim(current_time - 30, current_time)
                    else:
                        self.axes.flat[i].set_xlim(0, 30)

                    # 动态调整y轴范围
                    if len(self.wind_raw_data[i]) > 0:
                        all_data = list(self.wind_raw_data[i]) + list(self.wind_filtered_data[i])
                        y_min = min(all_data) - 1
                        y_max = max(all_data) + 1
                        if y_min < 0: y_min = 0
                        if y_max > 25: y_max = 25
                        self.axes.flat[i].set_ylim(y_min, y_max)

            # 更新成功信息
            success_rate = self.success_count / self.read_count * 100 if self.read_count > 0 else 0
            self.fig.suptitle(
                f'风速实时监测 | 成功率: {success_rate:.1f}% | '
                f'读取次数: {self.read_count} | '
                f'运行时间: {current_time:.0f}秒',
                fontsize=14, fontweight='bold'
            )

            # 打印最新数据
            print(f"\r时间: {current_time:6.1f}s | "
                  f"风速: {wind_data[0][1]:5.2f} | {wind_data[1][1]:5.2f} | "
                  f"{wind_data[2][1]:5.2f} | {wind_data[3][1]:5.2f} m/s", end='')
        else:
            self.read_count += 1
            self.fail_count += 1

        return self.lines_raw + self.lines_filtered

    def start(self):
        """启动实时绘图"""
        print("\n" + "="*60)
        print("风速实时监测系统")
        print("="*60)
        print(f"设备地址: {self.DEVICE_IP}:{self.DEVICE_PORT}")
        print(f"显示内容: 4个风速传感器的原始值和卡尔曼滤波值")
        print(f"数据点数: 显示最近{self.max_points}个数据点")
        print("按 Ctrl+C 停止程序")
        print("="*60)

        # 创建动画
        self.ani = FuncAnimation(
            self.fig, self.update, interval=100,  # 100ms更新一次
            blit=True, cache_frame_data=False
        )

        try:
            plt.show()
        except KeyboardInterrupt:
            print("\n\n⚠️ 用户中断，正在停止...")
        finally:
            if self.sock:
                self.sock.close()

            # 打印统计报告
            total_runtime = time.time() - self.start_time
            success_rate = self.success_count / self.read_count * 100 if self.read_count > 0 else 0

            print("\n" + "="*60)
            print("统计报告")
            print("="*60)
            print(f"总运行时间: {total_runtime:.1f} 秒")
            print(f"总读取次数: {self.read_count}")
            print(f"成功次数: {self.success_count}")
            print(f"失败次数: {self.fail_count}")
            print(f"成功率: {success_rate:.1f}%")
            print("="*60)


if __name__ == "__main__":
    plotter = WindSpeedPlotter()
    plotter.start()