import matplotlib
matplotlib.use('Agg')  # 使用非交互式后端
import matplotlib.pyplot as plt
import numpy as np
import time
from datetime import datetime
import os

print("程序将在后台运行，定期保存图形为图片文件...")

# 创建保存目录
save_dir = "wind_speed_plots"
os.makedirs(save_dir, exist_ok=True)

# 模拟数据收集
data_counter = 0

while True:
    data_counter += 1
    current_time = datetime.now()

    # 创建新图形
    plt.figure(figsize=(12, 8))

    # 生成模拟数据
    x = np.linspace(0, 60, 100)
    y1 = np.random.normal(5, 1, 100)  # 原始值
    y2 = np.random.normal(5, 0.5, 100)  # 滤波值
    y3 = np.random.normal(5.2, 0.5, 100)  # 修正值

    # 创建4个子图
    for i in range(4):
        plt.subplot(2, 2, i+1)

        # 不同的随机偏移
        offset = i * 0.2
        plt.plot(x, y1 + offset, 'r-', label='原始值', alpha=0.7)
        plt.plot(x, y2 + offset, 'b-', label='滤波后', linewidth=2)
        plt.plot(x, y3 + offset, 'g-', label='修正后', linewidth=2)

        plt.title(f'风速传感器 {i+1}\n最新值: 原始={5.1+offset:.2f} | 滤波={5.0+offset:.2f} | 修正={5.2+offset:.2f} m/s')
        plt.xlabel('时间 (秒)')
        plt.ylabel('风速 (m/s)')
        plt.grid(True, alpha=0.3)
        plt.legend()
        plt.ylim(0, 10)

    plt.suptitle(f'风速实时监测 - {current_time.strftime("%Y-%m-%d %H:%M:%S")}\n'
                  f'压力: 95.5kPa | 湿度: 31.5%',
                  fontsize=14)

    plt.tight_layout()

    # 保存图片
    filename = os.path.join(save_dir, f'wind_speed_{current_time.strftime("%Y%m%d_%H%M%S")}.png')
    plt.savefig(filename, dpi=100, bbox_inches='tight')
    plt.close()

    print(f"[{current_time.strftime('%H:%M:%S')}] 已保存图片: {filename}")

    # 等待10秒
    time.sleep(10)