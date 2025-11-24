import serial
import struct

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class EncoderAngleHexNode(Node):
    def __init__(self):
        super().__init__('encoder_angle_hex_node')

        # ---- 파라미터 ----
        self.declare_parameter('port', '/dev/ttyACM0')   # 윈도우면 COM3 이런 식
        self.declare_parameter('baudrate', 115200)

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baudrate').get_parameter_value().integer_value

        # ---- 시리얼 오픈 ----
        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=baud,
                timeout=0.01  # 10ms
            )
            self.get_logger().info(f'Opened serial port: {port} @ {baud}')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port {port}: {e}')
            raise

        # ---- 퍼블리셔 ----
        self.pub_angle = self.create_publisher(Float32, 'encoder_angle', 10)

        # ---- 타이머 (시리얼 폴링) ----
        self.timer = self.create_timer(0.001, self.timer_callback)

    def timer_callback(self):
        # 버퍼에 쌓인 줄 다 비우기
        while True:
            try:
                line = self.ser.readline().decode('ascii', errors='ignore').strip()
            except serial.SerialException as e:
                self.get_logger().error(f'Serial error: {e}')
                return

            if not line:
                break  # 읽을 거 없으면 종료

            # 기대 포맷: ANG,XXXXXXXX
            if not line.startswith('ANG,'):
                # 필요하면 디버그용 로그 켜기
                # self.get_logger().warn(f'Unknown line: {line}')
                continue

            try:
                _, hex_str = line.split(',', 1)
                hex_str = hex_str.strip()
                if len(hex_str) != 8:
                    self.get_logger().warn(f'HEX length not 8: {hex_str}')
                    continue

                # HEX → float32 (big endian)
                raw_bytes = bytes.fromhex(hex_str)
                angle_rad = struct.unpack('!f', raw_bytes)[0]   # ! = network(big-endian)
            except Exception as e:
                self.get_logger().warn(f'Failed to parse line \"{line}\": {e}')
                continue

            msg = Float32()
            msg.data = float(angle_rad)
            self.pub_angle.publish(msg)

    def destroy_node(self):
        try:
            if hasattr(self, 'ser') and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EncoderAngleHexNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

