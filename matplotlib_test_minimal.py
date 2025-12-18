# 最简单的matplotlib测试
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import numpy as np
import time

print("matplotlib版本:", matplotlib.__version__)
print("后端:", matplotlib.get_backend())

# 创建最简单的图形
plt.figure(figsize=(10, 6))
plt.title('测试窗口 - 如果能看到这个，matplotlib工作正常', fontsize=14)

# 生成测试数据
x = np.linspace(0, 10, 100)
y1 = np.sin(x)
y2 = np.cos(x)
y3 = np.sin(x + np.pi/4)

# 绘制三条线
plt.plot(x, y1, 'r-', label='正弦波', linewidth=2)
plt.plot(x, y2, 'b-', label='余弦波', linewidth=2)
plt.plot(x, y3, 'g-', label='相位偏移正弦波', linewidth=2)

plt.xlabel('X轴')
plt.ylabel('Y轴')
plt.legend()
plt.grid(True)

# 添加文本提示
plt.text(5, -1.5, '这是测试窗口！', ha='center', fontsize=16, color='red')

print("\n窗口已创建！")
print("如果看不到窗口：")
print("1. 检查任务栏")
print("2. 按Alt+Tab")
print("3. 程序将在5秒后自动关闭...")

# 显示窗口
try:
    plt.show(block=True)
except:
    print("使用非阻塞模式...")
    plt.show(block=False)
    for i in range(50):
        plt.pause(0.1)

print("测试完成")