"""/battery を subscribe して電池電圧を表示するノード。"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16


def to_volts(raw: int) -> float:
    """ロボットが送る電池の値を電圧 [V] に変換する。

    Args:
        raw: /battery の data。電圧を 10 倍した整数（例：82）。

    Returns:
        電圧 [V]（例：8.2）。
    """
    return raw / 10


class BatteryMonitor(Node):
    """/battery を受け取るたびに電圧をログに出すノード。"""

    def __init__(self):
        super().__init__('battery_monitor')
        # create_subscription(型, トピック名, コールバック関数, キューサイズ)
        self.create_subscription(UInt16, '/battery', self.on_battery, 10)

    def on_battery(self, msg: UInt16):
        # メッセージが届くたびに呼ばれる。msg.data に生の値が入っている
        self.get_logger().info(f'{to_volts(msg.data):.1f} V')


def main():
    rclpy.init()
    node = BatteryMonitor()
    try:
        # コールバックを待ち続ける（Ctrl+C で抜ける）
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
