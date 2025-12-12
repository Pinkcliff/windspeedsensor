import socket
import time
from typing import List, Dict, Optional


# --------------------------
# 核心工具函数：Modbus RTU帧处理（不变）
# --------------------------
def modbus_crc(data: List[int]) -> List[int]:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    return [crc & 0xFF, (crc >> 8) & 0xFF]


def build_rtu_request(slave_addr: int, start_reg: int, reg_count: int, func_code: int = 0x04) -> bytes:
    # 支持0x03（保持寄存器）和0x04（输入寄存器）切换
    frame = [
        slave_addr,
        func_code,
        (start_reg >> 8) & 0xFF,
        start_reg & 0xFF,
        (reg_count >> 8) & 0xFF,
        reg_count & 0xFF
    ]
    crc = modbus_crc(frame)
    frame.extend(crc)
    return bytearray(frame)


def parse_rtu_response(response_bytes: bytes) -> Dict:
    response = list(response_bytes)
    if len(response) < 4:
        return {"error": "响应帧过短"}

    slave_addr = response[0]
    func_code = response[1]
    data = response[2:-2]
    received_crc = response[-2:]

    calculated_crc = modbus_crc(response[:-2])
    if received_crc != calculated_crc:
        return {"error": f"CRC校验失败（接收: {received_crc}，计算: {calculated_crc}）"}

    if func_code in [0x03, 0x04]:
        if len(data) < 1:
            return {"error": f"功能码{func_code:02X}响应数据为空"}
        byte_count = data[0]
        registers = []
        for i in range(1, len(data), 2):
            if i + 1 > len(data):
                break
            reg_value = (data[i] << 8) | data[i + 1]
            registers.append(reg_value)
        return {
            "slave_addr": slave_addr,
            "func_code": func_code,
            "registers": registers,
            "valid": True
        }
    else:
        return {"error": f"不支持的功能码：0x{func_code:02X}"}


# --------------------------
# 优化模式：单次连接+持续读取+数据变化高亮+异常处理
# --------------------------
def single_connect_continuous_read():
    # 1. 设备参数（务必与设备手册一致！）
    DEVICE_IP = "192.168.0.101"    # 设备IP
    DEVICE_PORT = 8234           # 设备Modbus端口
    SLAVE_ADDR = 1               # 设备从站地址
    FUNC_CODE = 0x04             # 功能码（0x03=保持寄存器，0x04=输入寄存器）
    START_REG = 0                # 起始寄存器地址
    REG_COUNT = 2                # 读取寄存器数量
    READ_INTERVAL = 1            # 读取间隔（秒）
    TIMEOUT = 5                  # 单次读写超时时间
    BUFFER_SIZE = 1024
    RECONNECT_ATTEMPT = 1        # 连接断开后的重连次数

    # 2. 全局变量
    last_temperature: Optional[float] = None
    last_pressure: Optional[float] = None
    last_registers: List[int] = []
    read_count = 0               # 总读取次数
    success_count = 0            # 成功次数
    fail_count = 0               # 失败次数
    sock: Optional[socket.socket] = None  # 连接对象
    start_time = time.time()     # 程序开始时间

    # 3. 颜色编码
    RED = "\033[91m"
    GREEN = "\033[92m"
    YELLOW = "\033[93m"
    RESET = "\033[0m"

    print("="*80)
    print("🚀 启动 [单次连接+持续读取] 模式")
    print("="*80)
    print(f"📡 设备地址: {DEVICE_IP}:{DEVICE_PORT}")
    print(f"🔌 从站地址: {SLAVE_ADDR} | 功能码: 0x{FUNC_CODE:02X}")
    print(f"📝 读取范围: 寄存器{START_REG}~{START_REG+REG_COUNT-1}（共{REG_COUNT}个）")
    print(f"⏱️  读取间隔: {READ_INTERVAL}秒 | 超时时间: {TIMEOUT}秒")
    print(f"🔍 数据变化将以 {RED}红色{RESET} 高亮显示")
    print(f"🔄 连接断开后自动重连（{RECONNECT_ATTEMPT}次）")
    print("⛔ 按 Ctrl+C 停止读取")
    print("="*80)

    # 4. 连接函数（独立封装，方便重连）
    def connect_device() -> bool:
        """建立设备连接"""
        nonlocal sock
        try:
            # 关闭原有连接（如果存在）
            if sock:
                try:
                    sock.close()
                except:
                    pass
            
            # 创建新连接
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(TIMEOUT)
            sock.connect((DEVICE_IP, DEVICE_PORT))
            print(f"{GREEN}✅ 连接成功！{RESET}")
            return True
        except ConnectionRefusedError:
            print(f"{RED}❌ 连接失败: 设备拒绝连接（IP/端口错误或设备离线）{RESET}")
        except TimeoutError:
            print(f"{RED}❌ 连接失败: 连接超时{RESET}")
        except OSError as e:
            print(f"{RED}❌ 连接失败: 网络错误 - {str(e)}{RESET}")
        except Exception as e:
            print(f"{RED}❌ 连接失败: 未知错误 - {str(e)}{RESET}")
        return False

    # 5. 首次连接
    print(f"\n📞 正在连接设备 {DEVICE_IP}:{DEVICE_PORT}...")
    if not connect_device():
        print(f"{YELLOW}⚠️  连接失败，程序退出{RESET}")
        return

    print("-"*80)

    # 6. 持续读取循环
    try:
        while True:
            read_count += 1
            current_time = time.strftime("%H:%M:%S", time.localtime())
            read_start_time = time.time()
            read_success = False

            try:
                if not sock:
                    print(f"[{current_time}] {YELLOW}⚠️  无有效连接，尝试重连...{RESET}")
                    if not connect_device():
                        fail_count += 1
                        time.sleep(READ_INTERVAL)
                        continue

                # 发送请求
                request = build_rtu_request(
                    slave_addr=SLAVE_ADDR,
                    start_reg=START_REG,
                    reg_count=REG_COUNT,
                    func_code=FUNC_CODE
                )
                sock.sendall(request)

                # 接收响应
                response_bytes = b""
                request_start_time = time.time()
                
                while True:
                    chunk = sock.recv(BUFFER_SIZE)
                    if chunk:
                        response_bytes += chunk
                        # 检查完整帧
                        if len(response_bytes) >= 5:
                            data_len = response_bytes[2]
                            full_frame_len = 1 + 1 + 1 + data_len + 2  # 地址+功能码+字节数+数据+CRC
                            if len(response_bytes) >= full_frame_len:
                                break
                    
                    # 超时判断
                    if time.time() - request_start_time > TIMEOUT:
                        raise socket.timeout(f"接收超时（{TIMEOUT}秒）")
                    time.sleep(0.01)

                # 解析响应
                parsed_data = parse_rtu_response(response_bytes)
                if "error" in parsed_data:
                    print(f"[{current_time}] ❌ 第{read_count:03d}次: 解析失败 - {parsed_data['error']}")
                    fail_count += 1
                    time.sleep(READ_INTERVAL)
                    continue

                # 提取数据
                registers = parsed_data["registers"]
                if len(registers) < REG_COUNT:
                    print(f"[{current_time}] ❌ 第{read_count:03d}次: 数据不足（实际{len(registers)}个，期望{REG_COUNT}个）")
                    fail_count += 1
                    time.sleep(READ_INTERVAL)
                    continue

                # 数据转换
                temp_raw = registers[0]
                pressure_raw = registers[1]
                temperature = ((temp_raw / 249) - 4) * 7.5 - 40
                pressure = ((pressure_raw / 249) - 4) * 7.5
                read_duration = (time.time() - read_start_time) * 1000  # 毫秒

                # 高亮变化数据
                temp_str = f"{temperature:5.1f}℃"
                pressure_str = f"{pressure:6.1f}kPa"
                temp_raw_str = f"{temp_raw:4d}"
                pressure_raw_str = f"{pressure_raw:4d}"

                if last_temperature is not None and abs(temperature - last_temperature) > 0.1:
                    temp_str = f"{RED}{temp_str}{RESET}"
                    temp_raw_str = f"{RED}{temp_raw_str}{RESET}"
                
                if last_pressure is not None and abs(pressure - last_pressure) > 0.1:
                    pressure_str = f"{RED}{pressure_str}{RESET}"
                    pressure_raw_str = f"{RED}{pressure_raw_str}{RESET}"

                # 打印结果
                print(f"[{current_time}] ✅ 第{read_count:03d}次 | 耗时:{read_duration:4.0f}ms | "
                      f"温度原始值:{temp_raw_str} → {temp_str} | "
                      f"气压原始值:{pressure_raw_str} → {pressure_str}")

                # 更新记录
                last_temperature = temperature
                last_pressure = pressure
                last_registers = registers.copy()
                success_count += 1
                read_success = True

            except socket.timeout as e:
                print(f"[{current_time}] ⏰ 第{read_count:03d}次: 读取超时 - {str(e)}")
                fail_count += 1
            except ConnectionResetError:
                print(f"[{current_time}] {RED}🚫 第{read_count:03d}次: 连接被设备重置{RESET}")
                # 尝试重连
                reconnect_success = False
                for attempt in range(RECONNECT_ATTEMPT):
                    print(f"[{current_time}] 🔄 正在重连（{attempt+1}/{RECONNECT_ATTEMPT}）...")
                    if connect_device():
                        reconnect_success = True
                        break
                    time.sleep(2)  # 重连间隔
                if not reconnect_success:
                    print(f"[{current_time}] {RED}❌ 重连失败，程序将退出{RESET}")
                    break
                fail_count += 1
            except OSError as e:
                print(f"[{current_time}] {RED}❌ 第{read_count:03d}次: 网络错误 - {str(e)}{RESET}")
                fail_count += 1
            except Exception as e:
                # 捕获所有其他异常，避免程序崩溃
                print(f"[{current_time}] {RED}❌ 第{read_count:03d}次: 未知异常 - {str(e)}（{type(e).__name__}）{RESET}")
                fail_count += 1

            # 等待下一次读取
            time.sleep(READ_INTERVAL)

    except KeyboardInterrupt:
        print(f"\n{YELLOW}⚠️  用户中断，正在停止程序...{RESET}")
    finally:
        # 关闭连接
        if sock:
            try:
                sock.close()
                print(f"{GREEN}🔌 连接已关闭{RESET}")
            except:
                pass

    # 7. 最终统计报告
    total_runtime = time.time() - start_time
    success_rate = (success_count / read_count * 100) if read_count > 0 else 0.0

    print("\n" + "="*80)
    print("📋 读取结束 - 统计报告")
    print("="*80)
    print(f"🕐 总运行时间: {total_runtime:.1f} 秒")
    print(f"🔢 总读取次数: {read_count}")
    print(f"✅ 成功次数: {success_count}")
    print(f"❌ 失败次数: {fail_count}")
    print(f"📈 成功率: {success_rate:.1f}%")
    print("="*80)


if __name__ == "__main__":
    single_connect_continuous_read()