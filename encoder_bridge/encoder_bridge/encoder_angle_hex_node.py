import serial

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class EncoderAngleAsciiNode(Node):
    def __init__(self):
        super().__init__('encoder_angle_ascii_node')

        # ---- 파라미터 ----
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baudrate').get_parameter_value().integer_value

        # ---- 시리얼 오픈 ----
        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=baud,
                timeout=0.01
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
        while True:
            try:
                line = self.ser.readline().decode('ascii', errors='ignore').strip()
            except serial.SerialException as e:
                self.get_logger().error(f'Serial error: {e}')
                return

            if not line:
                break

            # 기대 포맷: ANG,<float>
            if not line.startswith('ANG,'):
                # self.get_logger().warn(f'Unknown line: {line}')
                continue

            try:
                _, val_str = line.split(',', 1)
                angle = float(val_str)
            except Exception as e:
                self.get_logger().warn(f'Failed to parse line \"{line}\": {e}')
                continue

            msg = Float32()
            msg.data = angle
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
    node = EncoderAngleAsciiNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

