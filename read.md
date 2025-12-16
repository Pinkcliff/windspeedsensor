主要功能：

  1. 数据自动保存 (plot_wind_speed.py)：

  - 每次运行程序时，会在 wind_data 目录下创建一个CSV文件
  - 文件名格式：YYYYMMDD_HHMMSS_wind_data.csv
  - 例如：20231216_143022_wind_data.csv

  2. 保存的数据包括：

  - 时间戳：精确到毫秒的完整时间戳
  - 运行时间：从程序开始的秒数
  - 风速数据：4个传感器的原始值和滤波值
  - 温度数据：温度值（°C）
  - 压力数据：压力值（kPa）
  - 空气密度：计算得到的空气密度（kg/m³）
  - 成功标志：1表示成功读取，0表示失败

  3. 数据查看工具 (view_data.py)：

  - 列出所有可用的数据文件
  - 选择文件进行查看
  - 显示数据统计信息
  - 生成可视化图表
  - 保存分析结果

  使用方法：

  运行数据采集程序：

  python plot_wind_speed.py
  程序会自动创建数据文件并保存数据。

  查看历史数据：

  python view_data.py
  程序会列出所有数据文件，你可以选择查看：
  - 详细的风速数据图
  - 统计摘要图
  - 数据质量分析

  CSV文件格式示例：

  Timestamp,Time_Elapsed_s,Wind1_Raw,Wind1_Filtered,Wind2_Raw,Wind2_Filtered,Wind3_Raw,Wind3_Filtered,Wind4_Raw,Wind4_Filtered,Temperature,Pressure,Air_Density,Success_Flag
  2023-12-16 14:30:22.123,0.10,5.23,5.18,8.45,8.30,6.12,6.08,4.98,4.95,22.5,101.3,1.18,1

  文件组织：

  windspeedsensor/
  ├── plot_wind_speed.py      # 主程序（带数据保存）
  ├── view_data.py           # 数据查看工具
  └── wind_data/             # 数据文件夹
      ├── 20231216_143022_wind_data.csv
      ├── 20231216_151245_wind_data.csv
      └── ...

  现在每次运行程序时，所有数据都会被保存，方便后续分析！