"""
卡尔曼滤波器模块
用于对传感器数据进行平滑处理，减少噪声和波动
"""

import numpy as np


class KalmanFilter:
    """一维卡尔曼滤波器"""

    def __init__(self, process_variance=1e-4, measurement_variance=5e-1, initial_estimate=0.0, initial_error=1.0):
        """
        初始化卡尔曼滤波器

        参数:
            process_variance: 过程噪声协方差 (Q) - 系统模型的不确定性
            measurement_variance: 测量噪声协方差 (R) - 传感器测量的不确定性
            initial_estimate: 初始状态估计值
            initial_error: 初始估计误差协方差 (P)
        """
        self.Q = process_variance  # 过程噪声
        self.R = measurement_variance  # 测量噪声
        self.X = initial_estimate  # 当前状态估计
        self.P = initial_error  # 当前估计误差协方差

        # 用于调试和监控
        self.kalman_gain = 0.0  # 卡尔曼增益

    def update(self, measurement):
        """
        更新滤波器状态

        参数:
            measurement: 新的测量值

        返回:
            filtered_value: 滤波后的值
        """
        # 预测步骤
        # X_pred = X (假设状态不变，因为我们对静态数据进行滤波)
        # P_pred = P + Q
        self.P = self.P + self.Q

        # 更新步骤
        # 计算卡尔曼增益
        self.kalman_gain = self.P / (self.P + self.R)

        # 更新状态估计
        self.X = self.X + self.kalman_gain * (measurement - self.X)

        # 更新误差协方差
        self.P = (1 - self.kalman_gain) * self.P

        return self.X

    def get_filtered_value(self):
        """获取当前滤波后的值"""
        return self.X

    def get_kalman_gain(self):
        """获取当前卡尔曼增益（用于调试）"""
        return self.kalman_gain

    def reset(self, initial_estimate=0.0, initial_error=1.0):
        """重置滤波器状态"""
        self.X = initial_estimate
        self.P = initial_error
        self.kalman_gain = 0.0


class MultiKalmanFilter:
    """多维卡尔曼滤波器管理类"""

    def __init__(self, num_filters, process_variance=1e-3, measurement_variance=1e-1):
        """
        初始化多个卡尔曼滤波器

        参数:
            num_filters: 滤波器数量
            process_variance: 过程噪声协方差
            measurement_variance: 测量噪声协方差
        """
        self.filters = []
        for i in range(num_filters):
            self.filters.append(
                KalmanFilter(
                    process_variance=process_variance,
                    measurement_variance=measurement_variance,
                    initial_estimate=0.0,
                    initial_error=1.0
                )
            )

    def update(self, measurements):
        """
        更新所有滤波器

        参数:
            measurements: 测量值列表

        返回:
            filtered_values: 滤波后的值列表
        """
        if len(measurements) != len(self.filters):
            raise ValueError("测量值数量与滤波器数量不匹配")

        filtered_values = []
        for i, (filter, measurement) in enumerate(zip(self.filters, measurements)):
            filtered_value = filter.update(measurement)
            filtered_values.append(filtered_value)

        return filtered_values

    def get_filtered_values(self):
        """获取所有滤波器的当前值"""
        return [f.get_filtered_value() for f in self.filters]

    def reset(self):
        """重置所有滤波器"""
        for f in self.filters:
            f.reset()

    def set_parameters(self, process_variance=None, measurement_variance=None):
        """设置滤波器参数"""
        if process_variance is not None:
            for f in self.filters:
                f.Q = process_variance
        if measurement_variance is not None:
            for f in self.filters:
                f.R = measurement_variance


def create_temperature_filter(initial_temp=20.0):
    """
    创建专门用于温度的卡尔曼滤波器

    参数:
        initial_temp: 初始温度估计值

    返回:
        KalmanFilter实例
    """
    # 温度滤波参数调整
    # process_variance: 温度变化通常较缓慢，使用较小的值
    # measurement_variance: 温度传感器测量噪声
    return KalmanFilter(
        process_variance=1e-4,  # 温度变化缓慢
        measurement_variance=0.5,  # 温度测量噪声
        initial_estimate=initial_temp,
        initial_error=1.0
    )


def create_pressure_filter(initial_pressure=100.0):
    """
    创建专门用于压力的卡尔曼滤波器

    参数:
        initial_pressure: 初始压力估计值

    返回:
        KalmanFilter实例
    """
    return KalmanFilter(
        process_variance=1e-3,
        measurement_variance=0.8,
        initial_estimate=initial_pressure,
        initial_error=1.0
    )


def create_wind_speed_filter(initial_wind_speed=0.0):
    """
    创建专门用于风速的卡尔曼滤波器

    参数:
        initial_wind_speed: 初始风速估计值

    返回:
        KalmanFilter实例
    """
    return KalmanFilter(
        process_variance=1e-4,  # 风速变化可能较快
        measurement_variance=0.1,  # 风速测量噪声较大
        initial_estimate=initial_wind_speed,
        initial_error=1.0
    )


# 示例使用代码
if __name__ == "__main__":
    import random

    # 模拟温度数据（带噪声）
    true_temperature = 25.0
    measurements = []
    filtered = []

    # 创建温度滤波器
    temp_filter = create_temperature_filter(true_temperature)

    print("卡尔曼滤波器测试:")
    print("真实温度: {:.2f}°C".format(true_temperature))
    print("-" * 50)

    for i in range(20):
        # 添加随机噪声
        noise = random.gauss(0, 1.0)  # 标准差为1.0的高斯噪声
        measurement = true_temperature + noise
        measurements.append(measurement)

        # 应用卡尔曼滤波
        filtered_value = temp_filter.update(measurement)
        filtered.append(filtered_value)

        print(f"第{i:2d}次: 原始值={measurement:6.2f}°C, 滤波值={filtered_value:6.2f}°C, 增益={temp_filter.get_kalman_gain():.4f}")

    # 计算统计信息
    raw_std = np.std(measurements)
    filtered_std = np.std(filtered[-10:])  # 使用后10个值计算稳定后的标准差

    print("-" * 50)
    print(f"原始数据标准差: {raw_std:.3f}")
    print(f"滤波后标准差: {filtered_std:.3f}")
    print(f"噪声减少: {((raw_std - filtered_std) / raw_std * 100):.1f}%")


# ==============================================================================
# 增强型滤波器 - 用于风速数据处理
# ==============================================================================

class StrongSmoothingFilter:
    """
    强化平滑滤波器
    通过调整Q/R参数实现更平滑的滤波效果
    适用于抖动严重的数据
    """
    def __init__(self, initial_wind_speed=0.0):
        self.kalman = KalmanFilter(
            process_variance=5e-5,      # 极小的过程噪声
            measurement_variance=10.0,  # 很大的测量噪声
            initial_estimate=initial_wind_speed,
            initial_error=1.0
        )

    def update(self, measurement):
        return self.kalman.update(measurement)


class DualStageFilter:
    """
    双级滤波器
    第一级快速去除高频噪声，第二级深度平滑
    适用于需要综合优化的场景
    """
    def __init__(self, initial_wind_speed=0.0):
        # 第一级：快速响应
        self.stage1 = KalmanFilter(
            process_variance=5e-4,
            measurement_variance=1.0,
            initial_estimate=initial_wind_speed
        )
        # 第二级：深度平滑
        self.stage2 = KalmanFilter(
            process_variance=1e-5,
            measurement_variance=5.0,
            initial_estimate=initial_wind_speed
        )

    def update(self, measurement):
        stage1_output = self.stage1.update(measurement)
        return self.stage2.update(stage1_output)


class RobustOutlierFilter:
    """
    自适应卡尔曼滤波器
    根据采样值与滤波值的差异动态调整参数：
    - 差异 > M值：使用大Q、大R快速逼近
    - 差异 ≤ M值：使用小Q、小R强力平滑
    """
    def __init__(self, initial_wind_speed=0.0, max_diff=1.0):
        """
        参数:
            initial_wind_speed: 初始风速估计值
            max_diff: 采样值与滤波值的最大允许差异（M值）
        """
        self.max_diff = max_diff

        # 快速逼近参数（差异大时使用）
        self.Q_fast = 5e-2    # 大Q：快速响应
        self.R_fast = 5.0     # 大R：对测量值中等信任

        # 强力平滑参数（差异小时使用）
        self.Q_slow = 1e-4    # 小Q：强力平滑
        self.R_slow = 1.0     # 小R：对测量值高信任

        # 初始化卡尔曼滤波器（使用快速参数开始）
        self.kalman = KalmanFilter(
            process_variance=self.Q_fast,
            measurement_variance=self.R_fast,
            initial_estimate=initial_wind_speed
        )
        self.last_filtered_value = initial_wind_speed

    def update(self, measurement):
        """
        更新滤波器
        参数:
            measurement: 新的采样值
        返回:
            滤波后的值
        """
        # 计算采样值与当前滤波值的差异
        diff = abs(measurement - self.last_filtered_value)

        # 根据差异动态调整Q和R参数
        if diff > self.max_diff:
            # 差异大：快速逼近模式
            self.kalman.Q = self.Q_fast
            self.kalman.R = self.R_fast
        else:
            # 差异小：强力平滑模式
            self.kalman.Q = self.Q_slow
            self.kalman.R = self.R_slow

        # 执行卡尔曼滤波更新
        filtered = self.kalman.update(measurement)
        self.last_filtered_value = filtered

        return filtered