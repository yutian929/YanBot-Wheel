#!/usr/bin/env python
import socket
import json
import csv
from datetime import datetime
from json.decoder import JSONDecodeError
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
import threading


class SensorDataServer:
    def __init__(self):
        # ROS节点初始化
        rospy.init_node("imu_iphone_data", anonymous=True)
        self.imu_pub = rospy.Publisher("/topic_imu", Imu, queue_size=10)
        self.frame_id = rospy.get_param(
            "~imu_frame_id", "imu_link"
        )  # 可从参数服务器获取frame_id

        # 网络通信参数
        self.server_port = 54321
        self.buffer_size = 4096
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

        # 注册ROS关闭钩子
        rospy.on_shutdown(self.shutdown)

    def shutdown(self):
        """ROS关闭时的清理操作"""
        self.running = False
        rospy.loginfo("正在关闭传感器数据服务器...")

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
        rospy.loginfo(f"\n=== 传感器数据接收服务器 ===")
        rospy.loginfo(f"本机IP: {host_ip}")
        rospy.loginfo(f"监听端口: {self.server_port}")
        rospy.loginfo("等待手机连接... (Ctrl+C 退出)")

        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_socket.bind(("0.0.0.0", self.server_port))
        server_socket.listen(1)
        server_socket.settimeout(1)  # 设置超时以定期检查运行状态

        try:
            while self.running:
                try:
                    client_socket, addr = server_socket.accept()
                    rospy.loginfo(f"\n新的连接来自: {addr[0]}")
                    self.handle_client(client_socket)
                except socket.timeout:
                    continue
        except Exception as e:
            rospy.logwarn(f"服务器异常: {str(e)}")
        finally:
            server_socket.close()

    def handle_client(self, client_socket):
        """处理客户端连接"""
        try:
            while self.running:
                data = client_socket.recv(self.buffer_size)
                if not data:
                    break

                self.data_buffer += data
                while True:
                    start_idx = self.data_buffer.find(b"{")
                    end_idx = (
                        self.data_buffer.find(b"}", start_idx + 1)
                        if start_idx != -1
                        else -1
                    )
                    if start_idx == -1 or end_idx == -1:
                        break

                    json_data = self.data_buffer[start_idx : end_idx + 1]
                    self.data_buffer = self.data_buffer[end_idx + 1 :]
                    self.process_json(json_data)

        except (ConnectionResetError, BrokenPipeError):
            rospy.logwarn("客户端连接异常断开")
        except Exception as e:
            rospy.logerr(f"处理数据时发生错误: {str(e)}")
        finally:
            client_socket.close()

    def process_json(self, json_bytes):
        """解析JSON数据并发布IMU消息"""
        try:
            data = json.loads(json_bytes.decode("utf-8"))
            if "motionPitch" not in data:
                return

            # 转换传感器数据
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

            # 创建并填充IMU消息
            imu_msg = Imu()
            imu_msg.header = Header()
            imu_msg.header.stamp = rospy.Time.from_sec(sensor_data["timestamp"])
            imu_msg.header.frame_id = self.frame_id

            # 四元数数据
            imu_msg.orientation.w = sensor_data["quat_w"]
            imu_msg.orientation.x = sensor_data["quat_x"]
            imu_msg.orientation.y = sensor_data["quat_y"]
            imu_msg.orientation.z = sensor_data["quat_z"]

            # 角速度数据（单位：rad/s）
            imu_msg.angular_velocity.x = sensor_data["gyro_x"]
            imu_msg.angular_velocity.y = sensor_data["gyro_y"]
            imu_msg.angular_velocity.z = sensor_data["gyro_z"]

            # 线加速度数据（单位：m/s²）
            imu_msg.linear_acceleration.x = sensor_data["accel_x"]
            imu_msg.linear_acceleration.y = sensor_data["accel_y"]
            imu_msg.linear_acceleration.z = sensor_data["accel_z"]

            # 协方差矩阵（-1表示未知）
            imu_msg.orientation_covariance[0] = -1
            imu_msg.angular_velocity_covariance = [0] * 9
            imu_msg.linear_acceleration_covariance = [0] * 9

            # 发布IMU消息
            self.imu_pub.publish(imu_msg)

            # 保留原始数据记录功能
            self.print_sensor_data(sensor_data)
            # self.save_to_csv(sensor_data)

        except JSONDecodeError:
            rospy.logwarn("JSON解析失败，原始数据: %s", json_bytes)
        except ValueError as e:
            rospy.logerr(f"数据转换错误: {str(e)}")
        except KeyError as e:
            rospy.logerr(f"缺少必要字段: {str(e)}")

    def print_sensor_data(self, data):
        """格式化打印传感器数据（可选）"""
        rospy.logdebug(
            f"\n[{datetime.fromtimestamp(data['timestamp']).strftime('%H:%M:%S.%f')}]"
        )
        rospy.logdebug(
            f"姿态角: Pitch={data['pitch']:.4f}, Roll={data['roll']:.4f}, Yaw={data['yaw']:.4f}"
        )
        rospy.logdebug(
            f"加速度: X={data['accel_x']:.6f}, Y={data['accel_y']:.6f}, Z={data['accel_z']:.6f} m/s²"
        )
        rospy.logdebug(
            f"陀螺仪: X={data['gyro_x']:.6f}, Y={data['gyro_y']:.6f}, Z={data['gyro_z']:.6f} rad/s"
        )
        rospy.logdebug(
            f"四元数: W={data['quat_w']:.4f}, X={data['quat_x']:.4f}, Y={data['quat_y']:.4f}, Z={data['quat_z']:.4f}"
        )

    def save_to_csv(self, data):
        """保存数据到CSV文件（可选）"""
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

    # 在独立线程中运行TCP服务器
    server_thread = threading.Thread(target=server.start_server)
    server_thread.start()

    # 主线程运行ROS spin
    rospy.spin()

    # ROS关闭后清理
    server.running = False
    server_thread.join()
