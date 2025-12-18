import csv
import time
from datetime import datetime

def save_wind_data_to_csv(filename, data):
    """保存风速数据到CSV文件"""
    with open(filename, 'a', newline='') as file:
        writer = csv.writer(file)

        # 如果文件是新的，写入标题
        if file.tell() == 0:
            writer.writerow([
                '时间戳',
                'RTD05_原始', 'RTD05_滤波', 'RTD05_修正',
                'RTD06_原始', 'RTD06_滤波', 'RTD06_修正',
                'RTD07_原始', 'RTD07_滤波', 'RTD07_修正',
                'RTD08_原始', 'RTD08_滤波', 'RTD08_修正'
            ])

        # 写入数据
        writer.writerow([
            datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3],
            data[0][0], data[0][1], data[0][2],  # RTD05
            data[1][0], data[1][1], data[1][2],  # RTD06
            data[2][0], data[2][1], data[2][2],  # RTD07
            data[3][0], data[3][1], data[3][2]   # RTD08
        ])

# 示例使用
if __name__ == "__main__":
    # 生成测试数据
    import random
    test_data = []
    for i in range(4):
        test_data.append((
            random.uniform(4, 6),  # 原始
            random.uniform(4.5, 5.5),  # 滤波
            random.uniform(5, 5.8)  # 修正
        ))

    # 保存数据
    save_wind_data_to_csv("wind_data.csv", test_data)
    print("数据已保存到 wind_data.csv")