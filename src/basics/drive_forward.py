"""/cmd_vel を publish して、ロボットを一定時間だけ前進させて止めるノード。"""
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

SPEED = 0.1          # 前進速度 [m/s]
DURATION = 1.0       # 前進する時間 [s]
PUBLISH_RATE = 10.0  # 指令を送る周期 [Hz]
STOP_TOLERANCE = 0.01  # これ未満の速度 [m/s, rad/s] なら停止とみなす
STOP_TIMEOUT = 2.0     # 停止を確認できなくても諦めるまでの時間 [s]


def velocity_at(elapsed: float, duration: float, speed: float) -> float:
    """経過時間に応じた前進速度を返す。

    Args:
        elapsed: 走り始めてからの経過時間 [s]。
        duration: 前進を続ける時間 [s]。
        speed: 前進中の速度 [m/s]。

    Returns:
        duration 未満なら speed、以降は 0.0。
    """
    return speed if elapsed < duration else 0.0


def is_stopped(linear_x: float, angular_z: float, tolerance: float = STOP_TOLERANCE) -> bool:
    """オドメトリの速度からロボットが止まっているかを判定する。

    Args:
        linear_x: 前後方向の速度 [m/s]。
        angular_z: 旋回の角速度 [rad/s]。
        tolerance: 停止とみなす速度の上限。

    Returns:
        前後・旋回の速度がどちらも tolerance 未満なら True。
    """
    return abs(linear_x) < tolerance and abs(angular_z) < tolerance


class DriveForward(Node):
    """DURATION 秒だけ SPEED で前進し、その後に止まるノード。"""

    def __init__(self):
        super().__init__('drive_forward')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.start = time.monotonic()
        self.done = False
        # 停止の確認に使う、車輪から計算した実際の速度（最新値だけ欲しいのでキューは 1）
        self.odom_speed = None
        self.create_subscription(Odometry, '/odom_raw', self.on_odom, 1)
        # ロボットは最後の指令を保持し、WiFi で1通落ちることもあるため、走行中も周期的に送り続ける
        self.timer = self.create_timer(1.0 / PUBLISH_RATE, self.on_timer)

    def on_odom(self, msg: Odometry):
        self.odom_speed = (msg.twist.twist.linear.x, msg.twist.twist.angular.z)

    def on_timer(self):
        elapsed = time.monotonic() - self.start
        msg = Twist()
        msg.linear.x = velocity_at(elapsed, DURATION, SPEED)
        self.publisher.publish(msg)
        self.done = elapsed >= DURATION

    def stop(self):
        """ロボットが止まったことをオドメトリで確認できるまで停止指令を送る。

        Side Effects:
            走行指令のタイマーを止め、/cmd_vel に速度 0 を周期的に publish する。
            STOP_TIMEOUT 秒以内に停止を確認できなければ警告をログに出す。
        """
        # 走行指令が停止指令と混ざらないように、先にタイマーを止める
        self.timer.cancel()
        # 停止指令を送る前の速度で判定しないように、古い値を捨てる
        self.odom_speed = None
        deadline = time.monotonic() + STOP_TIMEOUT
        while time.monotonic() < deadline:
            self.publisher.publish(Twist())
            rclpy.spin_once(self, timeout_sec=1.0 / PUBLISH_RATE)
            if self.odom_speed is not None and is_stopped(*self.odom_speed):
                self.get_logger().info('停止を確認しました')
                return
        self.get_logger().warning(f'{STOP_TIMEOUT} 秒以内に停止を確認できませんでした')


def main():
    # Humble 標準の Ctrl+C 処理は通信を先に閉じてしまい、停止指令を送れない。
    # そこで ROS の Ctrl+C 処理を無効にし、Python の KeyboardInterrupt で受ける
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = DriveForward()
    try:
        # spin() は Ctrl+C で抜けられなくなるため、短い spin_once を繰り返す
        while not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        # 正常終了でも Ctrl+C でも、必ず止めてから終わる
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
