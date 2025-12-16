"""
测试卡尔曼滤波效果
模拟温度数据并展示滤波前后的对比
"""

import numpy as np
import matplotlib.pyplot as plt
from kalman_filter import create_temperature_filter
import random

def simulate_temperature_data(n_points=200, true_temp=25.0, noise_std=1.0):
    """模拟带噪声的温度数据"""
    # 添加缓慢的温度变化
    time = np.linspace(0, 4*np.pi, n_points)
    temperature_variation = 2 * np.sin(time/2)  # 缓慢变化的温度

    # 生成真实温度数据
    true_temperatures = true_temp + temperature_variation

    # 添加测量噪声
    noisy_measurements = []
    for temp in true_temperatures:
        noise = random.gauss(0, noise_std)
        noisy_measurements.append(temp + noise)

    return true_temperatures, noisy_measurements

def test_kalman_filter():
    """测试卡尔曼滤波器"""
    # 生成模拟数据
    true_temps, noisy_measurements = simulate_temperature_data()

    # 创建卡尔曼滤波器
    kalman_filter = create_temperature_filter(initial_temp=20.0)

    # 应用卡尔曼滤波
    filtered_temps = []
    for measurement in noisy_measurements:
        filtered_temp = kalman_filter.update(measurement)
        filtered_temps.append(filtered_temp)

    # 计算统计信息
    noise_std = np.std(noisy_measurements)
    filtered_std = np.std(filtered_temps)
    true_std = np.std(true_temps)

    # 计算与真实值的误差
    mse_noisy = np.mean((np.array(noisy_measurements) - np.array(true_temps))**2)
    mse_filtered = np.mean((np.array(filtered_temps) - np.array(true_temps))**2)

    # 打印结果
    print("="*60)
    print("卡尔曼滤波效果测试")
    print("="*60)
    print(f"模拟数据点数: {len(true_temps)}")
    print(f"真实温度变化标准差: {true_std:.3f}°C")
    print(f"原始噪声标准差: {noise_std:.3f}°C")
    print(f"滤波后标准差: {filtered_std:.3f}°C")
    print(f"噪声减少: {((noise_std - filtered_std) / noise_std * 100):.1f}%")
    print(f"原始数据MSE: {mse_noisy:.3f}")
    print(f"滤波后MSE: {mse_filtered:.3f}")
    print(f"MSE改善: {((mse_noisy - mse_filtered) / mse_noisy * 100):.1f}%")
    print("="*60)

    # 显示前20个数据点对比
    print("\n前20个数据点对比:")
    print("序号 | 真实值   | 原始值   | 滤波值   | 滤波增益")
    print("-"*60)
    for i in range(min(20, len(true_temps))):
        print(f"{i:3d}  | {true_temps[i]:7.3f} | {noisy_measurements[i]:7.3f} | {filtered_temps[i]:7.3f} | {kalman_filter.get_kalman_gain():.4f}")

    # 绘制图表
    try:
        import matplotlib.pyplot as plt

        plt.figure(figsize=(12, 6))

        # 绘制三条线
        plt.plot(true_temps, 'g-', label='真实温度', linewidth=2, alpha=0.7)
        plt.plot(noisy_measurements, 'b-', label='原始测量', alpha=0.5, linewidth=0.8)
        plt.plot(filtered_temps, 'r-', label='卡尔曼滤波', linewidth=2)

        plt.xlabel('采样点')
        plt.ylabel('温度 (°C)')
        plt.title('卡尔曼滤波效果对比')
        plt.legend()
        plt.grid(True, alpha=0.3)

        # 添加注释
        plt.annotate('原始数据噪声较大', xy=(50, max(noisy_measurements[40:60])),
                    xytext=(100, max(noisy_measurements)+2),
                    arrowprops=dict(arrowstyle='->', color='blue'))

        plt.annotate('滤波后更平滑', xy=(150, filtered_temps[150]),
                    xytext=(100, min(true_temps)-2),
                    arrowprops=dict(arrowstyle='->', color='red'))

        plt.tight_layout()
        plt.show()

    except ImportError:
        print("\n注意: 未安装matplotlib，无法显示图表")
        print("可以使用以下命令安装: pip install matplotlib")

    return true_temps, noisy_measurements, filtered_temps

if __name__ == "__main__":
    test_kalman_filter()