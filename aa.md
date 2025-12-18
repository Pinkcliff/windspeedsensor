真实数据的程序是：rtd_wind_density_combined.py

  这个版本连接实际设备：
  - 风速设备: 192.168.0.101:8234
  - RTD设备: 192.168.1.101:8234

  而测试版本是：
  - rtd_wind_density_interactive.py - 模拟数据
  - rtd_wind_density_combined_sim.py - 模拟数据
  - rtd_wind_density_simple.py - 简单模拟

  要使用真实数据，运行：
  python rtd_wind_density_combined.py

  这个程序会：
  1. 连接到实际的传感器设备
  2. 读取真实的风速和温度数据
  3. 应用卡尔曼滤波
  4. 基于RTD温度计算空气密度修正
  5. 显示在带有交互式勾选框的图表中

  需要注意的参数：
  - 标定条件：23.1°C, 65%RH, 101.325kPa
  - 密度修正公式：K = √(标定密度/实时密度)
  - 修正后风速 = 滤波后风速 × K