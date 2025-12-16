"""
测试风速数据绘图
生成模拟数据并展示卡尔曼滤波效果
"""

import numpy as np
import matplotlib.pyplot as plt
from kalman_filter import create_wind_speed_filter
import random
from datetime import datetime, timedelta

def simulate_wind_data(n_points=200, duration=60):
    """
    模拟风速数据

    参数:
        n_points: 数据点数
        duration: 持续时间（秒）
    """
    # 生成时间轴
    time_points = np.linspace(0, duration, n_points)

    # 生成四个传感器的模拟数据
    wind_data = []

    # 传感器1: 稳定风速，带噪声
    base_wind = 5.0
    noise1 = np.random.normal(0, 0.5, n_points)
    wind1 = base_wind + noise1

    # 传感器2: 阵风模式
    wind2 = []
    for t in time_points:
        # 基础风速 + 阵风 + 噪声
        gust = 3 * np.sin(t/5) * (np.random.random() > 0.7)  # 30%概率出现阵风
        noise = np.random.normal(0, 0.8)
        wind2.append(8 + gust + noise)
    wind2 = np.array(wind2)

    # 传感器3: 缓慢变化的风速
    wind3 = 6 + 3 * np.sin(t/10) + np.random.normal(0, 0.6, n_points)

    # 传感器4: 突发性变化
    wind4 = []
    current_wind = 4
    for t in time_points:
        if random.random() < 0.02:  # 2%概率突变
            current_wind = random.uniform(2, 12)
        noise = np.random.normal(0, 0.4)
        wind4.append(current_wind + noise)
    wind4 = np.array(wind4)

    return time_points, [wind1, wind2, wind3, wind4]

def apply_kalman_filter(wind_data):
    """对风速数据应用卡尔曼滤波"""
    filtered_data = []

    for i, sensor_data in enumerate(wind_data):
        # 创建滤波器
        kalman_filter = create_wind_speed_filter()

        # 应用滤波
        filtered = []
        for value in sensor_data:
            filtered_value = kalman_filter.update(value)
            filtered.append(filtered_value)

        filtered_data.append(np.array(filtered))

    return filtered_data

def plot_wind_data(time_points, wind_data, filtered_data):
    """绘制风速数据"""
    # 创建图表
    fig, axes = plt.subplots(2, 2, figsize=(15, 10))
    fig.suptitle('风速传感器数据对比（原始值 vs 卡尔曼滤波值）', fontsize=16, fontweight='bold')

    colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728']
    sensor_labels = ['传感器 1 (稳定)', '传感器 2 (阵风)', '传感器 3 (渐变)', '传感器 4 (突变)']

    for idx, (ax, raw_data, filtered_data, color, label) in enumerate(
        zip(axes.flat, wind_data, filtered_data, colors, sensor_labels)):

        # 绘制原始数据
        ax.plot(time_points, raw_data, color=color, alpha=0.5, linewidth=0.8, label='原始值')

        # 绘制滤波后数据
        ax.plot(time_points, filtered_data, color=color, linewidth=2, label='滤波值')

        # 设置标题和标签
        ax.set_title(f'{label}', fontsize=12, fontweight='bold')
        ax.set_xlabel('时间 (秒)', fontsize=10)
        ax.set_ylabel('风速 (m/s)', fontsize=10)
        ax.grid(True, alpha=0.3)
        ax.legend(loc='upper right')

        # 计算统计信息
        raw_std = np.std(raw_data)
        filtered_std = np.std(filtered_data)
        noise_reduction = (raw_std - filtered_std) / raw_std * 100

        # 在图上显示统计信息
        stats_text = f'原始标准差: {raw_std:.3f} m/s\n'
        stats_text += f'滤波标准差: {filtered_std:.3f} m/s\n'
        stats_text += f'噪声减少: {noise_reduction:.1f}%'
        ax.text(0.02, 0.98, stats_text, transform=ax.transAxes,
                verticalalignment='top', bbox=dict(boxstyle='round', facecolor='white', alpha=0.8),
                fontsize=9)

    # 调整布局
    plt.tight_layout()
    plt.subplots_adjust(top=0.93)

    # 保存图片
    plt.savefig('wind_speed_comparison.png', dpi=300, bbox_inches='tight')
    print("图表已保存为 'wind_speed_comparison.png'")

    plt.show()

def plot_summary_statistics(time_points, wind_data, filtered_data):
    """绘制汇总统计图"""
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))

    # 计算平均值
    raw_avg = [np.mean(data) for data in wind_data]
    filtered_avg = [np.mean(data) for data in filtered_data]

    # 计算标准差
    raw_std = [np.std(data) for data in wind_data]
    filtered_std = [np.std(data) for data in filtered_data]

    sensor_labels = ['传感器1', '传感器2', '传感器3', '传感器4']
    x = np.arange(len(sensor_labels))
    width = 0.35

    # 平均值对比
    ax1.bar(x - width/2, raw_avg, width, label='原始值平均值', alpha=0.8)
    ax1.bar(x + width/2, filtered_avg, width, label='滤波值平均值', alpha=0.8)
    ax1.set_xlabel('传感器')
    ax1.set_ylabel('平均风速 (m/s)')
    ax1.set_title('平均风速对比')
    ax1.set_xticks(x)
    ax1.set_xticklabels(sensor_labels)
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    # 标准差对比
    ax2.bar(x - width/2, raw_std, width, label='原始值标准差', alpha=0.8, color='orange')
    ax2.bar(x + width/2, filtered_std, width, label='滤波值标准差', alpha=0.8, color='red')
    ax2.set_xlabel('传感器')
    ax2.set_ylabel('标准差 (m/s)')
    ax2.set_title('数据稳定性对比（标准差越小越稳定）')
    ax2.set_xticks(x)
    ax2.set_xticklabels(sensor_labels)
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    # 添加噪声减少百分比
    for i in range(len(sensor_labels)):
        reduction = (raw_std[i] - filtered_std[i]) / raw_std[i] * 100
        ax2.text(i, filtered_std[i] + 0.1, f'{reduction:.1f}%',
                ha='center', va='bottom', fontweight='bold')

    plt.tight_layout()
    plt.savefig('wind_speed_statistics.png', dpi=300, bbox_inches='tight')
    print("统计图表已保存为 'wind_speed_statistics.png'")
    plt.show()

def main():
    """主函数"""
    print("="*60)
    print("风速数据滤波效果测试")
    print("="*60)

    # 生成模拟数据
    print("生成模拟风速数据...")
    time_points, wind_data = simulate_wind_data(n_points=500, duration=120)

    # 应用卡尔曼滤波
    print("应用卡尔曼滤波...")
    filtered_data = apply_kalman_filter(wind_data)

    # 打印统计信息
    print("\n统计信息:")
    print("-"*60)
    for i in range(4):
        raw_std = np.std(wind_data[i])
        filtered_std = np.std(filtered_data[i])
        noise_reduction = (raw_std - filtered_std) / raw_std * 100

        print(f"传感器 {i+1}:")
        print(f"  原始数据标准差: {raw_std:.4f} m/s")
        print(f"  滤波数据标准差: {filtered_std:.4f} m/s")
        print(f"  噪声减少: {noise_reduction:.2f}%")
        print()

    # 绘制图表
    print("绘制对比图表...")
    plot_wind_data(time_points, wind_data, filtered_data)

    # 绘制统计图表
    print("绘制统计图表...")
    plot_summary_statistics(time_points, wind_data, filtered_data)

    print("\n测试完成！")

if __name__ == "__main__":
    main()