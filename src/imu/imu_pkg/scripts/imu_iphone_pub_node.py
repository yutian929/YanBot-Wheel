import socket
import json
import csv
from datetime import datetime
from json.decoder import JSONDecodeError


class SensorDataServer:
    def __init__(self):
        self.server_port = 54321
        self.buffer_size = 4096  # 适当增大缓冲区
        self.running = True
        self.data_buffer = b""

        # 初始化CSV文件
        with open("sensor_data.csv", "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(
                [
                    "timestamp",
                    "logging_time",
                    "pitch",
                    "roll",
                    "yaw",
                    "accel_x",
                    "accel_y",
                    "accel_z",
                    "gyro_x",
                    "gyro_y",
                    "gyro_z",
                    "mag_x",
                    "mag_y",
                    "mag_z",
                    "quat_w",
                    "quat_x",
                    "quat_y",
                    "quat_z",
                ]
            )

    def get_local_ip(self):
        """获取本机局域网IP地址"""
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            local_ip = s.getsockname()[0]
            s.close()
            return local_ip
        except Exception:
            return "0.0.0.0"

    def start_server(self):
        """启动传感器数据服务器"""
        host_ip = self.get_local_ip()
        print(f"\n\033[1;36m=== 传感器数据接收服务器 ===\033[0m")
        print(f"本机IP: \033[1;32m{host_ip}\033[0m")
        print(f"监听端口: {self.server_port}")
        print("等待手机连接... (Ctrl+C 退出)")

        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_socket.bind(("0.0.0.0", self.server_port))
        server_socket.listen(1)

        try:
            while self.running:
                client_socket, addr = server_socket.accept()
                print(f"\n\033[1;32m新的连接来自: {addr[0]}\033[0m")
                self.handle_client(client_socket)

        except KeyboardInterrupt:
            print("\n服务器正在关闭...")
        finally:
            server_socket.close()

    def handle_client(self, client_socket):
        """处理客户端连接"""
        try:
            while self.running:
                data = client_socket.recv(self.buffer_size)
                if not data:
                    break

                # 累积数据到缓冲区
                self.data_buffer += data

                # 处理缓冲区中的所有完整JSON对象
                while True:
                    # 查找JSON对象边界
                    start_idx = self.data_buffer.find(b"{")
                    end_idx = (
                        self.data_buffer.find(b"}", start_idx + 1)
                        if start_idx != -1
                        else -1
                    )

                    # 没有完整对象时退出循环
                    if start_idx == -1 or end_idx == -1:
                        break

                    # 提取并处理单个JSON对象
                    json_data = self.data_buffer[start_idx : end_idx + 1]
                    self.data_buffer = self.data_buffer[end_idx + 1 :]
                    self.process_json(json_data)

        except (ConnectionResetError, BrokenPipeError):
            print("\033[1;31m客户端连接异常断开\033[0m")
        except Exception as e:
            print(f"处理数据时发生错误: {str(e)}")
        finally:
            client_socket.close()

    def process_json(self, json_bytes):
        """解析和处理JSON数据"""
        try:
            # 转换JSON并验证基本结构
            data = json.loads(json_bytes.decode("utf-8"))
            if "motionPitch" not in data:
                return

            # 数据类型转换
            sensor_data = {
                "timestamp": float(data.get("msg-time-sent", 0)),
                "logging_time": data.get("loggingTime", ""),
                "pitch": float(data.get("motionPitch", 0)),
                "roll": float(data.get("motionRoll", 0)),
                "yaw": float(data.get("motionYaw", 0)),
                "accel_x": float(data.get("motionUserAccelerationX", 0)),
                "accel_y": float(data.get("motionUserAccelerationY", 0)),
                "accel_z": float(data.get("motionUserAccelerationZ", 0)),
                "gyro_x": float(data.get("motionRotationRateX", 0)),
                "gyro_y": float(data.get("motionRotationRateY", 0)),
                "gyro_z": float(data.get("motionRotationRateZ", 0)),
                "mag_x": float(data.get("motionMagneticFieldX", 0)),
                "mag_y": float(data.get("motionMagneticFieldY", 0)),
                "mag_z": float(data.get("motionMagneticFieldZ", 0)),
                "quat_w": float(data.get("motionQuaternionW", 0)),
                "quat_x": float(data.get("motionQuaternionX", 0)),
                "quat_y": float(data.get("motionQuaternionY", 0)),
                "quat_z": float(data.get("motionQuaternionZ", 0)),
            }

            # 打印数据摘要
            self.print_sensor_data(sensor_data)

            # 保存到CSV
            self.save_to_csv(sensor_data)

        except JSONDecodeError:
            print("JSON解析失败，原始数据:", json_bytes)
        except ValueError as e:
            print(f"数据转换错误: {str(e)}")
        except KeyError as e:
            print(f"缺少必要字段: {str(e)}")

    def print_sensor_data(self, data):
        """格式化打印传感器数据"""
        print(
            f"\n[{datetime.fromtimestamp(data['timestamp']).strftime('%H:%M:%S.%f')}]"
        )
        print(
            f"姿态角: Pitch={data['pitch']:.4f}, Roll={data['roll']:.4f}, Yaw={data['yaw']:.4f}"
        )
        print(
            f"加速度: X={data['accel_x']:.6f}, Y={data['accel_y']:.6f}, Z={data['accel_z']:.6f} m/s²"
        )
        print(
            f"陀螺仪: X={data['gyro_x']:.6f}, Y={data['gyro_y']:.6f}, Z={data['gyro_z']:.6f} rad/s"
        )
        print(
            f"四元数: W={data['quat_w']:.4f}, X={data['quat_x']:.4f}, Y={data['quat_y']:.4f}, Z={data['quat_z']:.4f}"
        )

    def save_to_csv(self, data):
        """保存数据到CSV文件"""
        with open("sensor_data.csv", "a", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(
                [
                    data["timestamp"],
                    data["logging_time"],
                    data["pitch"],
                    data["roll"],
                    data["yaw"],
                    data["accel_x"],
                    data["accel_y"],
                    data["accel_z"],
                    data["gyro_x"],
                    data["gyro_y"],
                    data["gyro_z"],
                    data["mag_x"],
                    data["mag_y"],
                    data["mag_z"],
                    data["quat_w"],
                    data["quat_x"],
                    data["quat_y"],
                    data["quat_z"],
                ]
            )


if __name__ == "__main__":
    server = SensorDataServer()
    server.start_server()
