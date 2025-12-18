import matplotlib.pyplot as plt
import numpy as np
import time

# 设置交互模式
plt.ion()

# 创建图形
fig, axes = plt.subplots(2, 2, figsize=(12, 8))
fig.suptitle('风速实时监测测试', fontsize=14)
axes = axes.flatten()

# 初始化数据
x = np.linspace(0, 10, 100)
lines = []

for i, ax in enumerate(axes):
    ax.set_title(f'传感器 {i+1}')
    ax.set_xlabel('时间(s)')
    ax.set_ylabel('风速(m/s)')
    ax.grid(True)

    # 创建三条线
    line1, = ax.plot([], [], 'r-', label='原始值')
    line2, = ax.plot([], [], 'b-', label='滤波后')
    line3, = ax.plot([], [], 'g-', label='修正后')
    lines.append((line1, line2, line3))
    ax.legend()

# 调整布局
plt.tight_layout()

# 显示窗口
plt.show()

print("测试窗口已创建！")
print("如果看到这个消息但看不到窗口，请：")
print("1. 检查任务栏")
print("2. 按Alt+Tab")
print("\n5秒后自动关闭...")

# 更新几次数据
for _ in range(50):
    for i in range(4):
        # 生成随机数据
        y_raw = np.random.randn(100) * 0.5 + 5
        y_filtered = np.random.randn(100) * 0.3 + 4.8
        y_corrected = np.random.randn(100) * 0.3 + 5.2

        lines[i][0].set_data(x, y_raw)
        lines[i][1].set_data(x, y_filtered)
        lines[i][2].set_data(x, y_corrected)

        axes[i].relim()
        axes[i].autoscale_view()

    plt.pause(0.1)
    time.sleep(0.1)

plt.ioff()
print("测试完成")