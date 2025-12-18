import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import numpy as np

print(f"matplotlib version: {matplotlib.__version__}")
print(f"matplotlib backend: {matplotlib.get_backend()}")

# 创建简单的测试图形
fig, ax = plt.subplots(figsize=(8, 6))
x = np.linspace(0, 10, 100)
y = np.sin(x)

ax.plot(x, y, 'b-', linewidth=2)
ax.set_title('Matplotlib测试 - 正弦波')
ax.set_xlabel('X轴')
ax.set_ylabel('Y轴')
ax.grid(True)

print("\n正在显示测试窗口...")
print("如果看不到窗口，请尝试以下解决方案：")
print("1. 安装tkinter: pip install tk")
print("2. 或者安装PyQt: pip install PyQt5")
print("3. 在Ubuntu/Debian上: sudo apt-get install python3-tk")

plt.show(block=True)
print("测试窗口已关闭")