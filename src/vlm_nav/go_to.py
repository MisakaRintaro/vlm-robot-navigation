"""言葉で指示した目標を Gemini Robotics-ER でカメラ画像から探し、近づいて止まるノード。

使い方（ros コンテナの中で）:
    python3 go_to.py "pink tape"

設計は docs/design.md を参照。
"""
import argparse
import json
import math
import random
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from google import genai
from google.genai import errors, types
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import CompressedImage, LaserScan
from std_msgs.msg import Int32, String, UInt16

import config
from nav_logic import (
    build_prompt,
    build_sweep_prompt,
    heading_to_capture,
    most_open_direction,
    open_directions,
    normalize_angle,
    parse_sweep,
    parse_target,
    sector_distance,
    target_bearing,
    yaw_from_quaternion,
)
from recorder import RunRecorder

# --- 調整しない値（調整するものは config.py） ---
MODEL = 'gemini-robotics-er-2-preview'
SENSOR_TIMEOUT = 1.0         # センサがこれ以上届かなければ停止 [s]
MOTION_TIMEOUT_FACTOR = 3.0  # 旋回・前進が想定時間のこの倍を超えたら停止
STARTUP_TIMEOUT = 10.0       # 起動時にセンサを待つ時間 [s]
RATE = 10.0                  # 制御周期 [Hz]
STOP_TOLERANCE = 0.01        # 停止とみなす速度 [m/s, rad/s]
STOP_TIMEOUT = 2.0           # 停止確認を諦める時間 [s]
API_RETRY_WAIT = 10.0        # API エラー（回数制限など）のあと、聞き直すまで待つ時間 [s]
# 首振り・旋回が終わってから、この時間より後に届いた画像だけを使う [s]。
# カメラ画像は WiFi 経由で遅れて届くため、直後の画像は動く前の向きのものであることがある
CAMERA_SETTLE = 1.0
RUNS_DIR = Path('/ws/runs')  # 実行記録の保存先
# LiDAR の値を有効とみなす最短距離 [m]。/scan の range_min は 0.12 だが、実機は 0.05m 程度まで安定して返す。
# range_min で捨てると 12cm より近い物が見えなくなるため、0.00 などの無効値だけを除く値にする
LIDAR_MIN_RANGE = 0.03
# LiDAR の中心から車体の端までの距離 [m]（端に物を当てて実測。カメラを LiDAR の測定面から外した状態）
FRONT_EDGE = 0.113
REAR_EDGE = 0.135


class SafetyStop(Exception):
    """安全のために動作を中断するときに送出する。"""


class GoTo(Node):
    """目標を探して近づくノード。センサの受信・走行指令・記録を持つ。"""

    def __init__(self, target: str):
        super().__init__('go_to')
        self.target = target
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.beep_pub = self.create_publisher(UInt16, '/beep', 10)
        self.event_pub = self.create_publisher(String, '/go_to/event', 10)
        self.servo_pubs = [self.create_publisher(Int32, topic, 10)
                           for topic in ('/servo_s1', '/servo_s2')]
        # 最新値だけ使うのでキューは 1
        self.create_subscription(Odometry, '/odom_raw', self.on_odom, 1)
        self.create_subscription(LaserScan, '/scan', self.on_scan, 1)
        self.create_subscription(CompressedImage, '/espRos/esp32camera', self.on_image, 1)
        self.odom = None
        self.scan = None
        self.jpeg = None
        self.image_time = 0.0          # 最新の画像が届いた時刻
        self.last_odom_time = 0.0
        self.last_scan_time = 0.0
        self.client = genai.Client()
        self.recorder = RunRecorder(RUNS_DIR)
        self.start_time = time.monotonic()
        self.queries = 0               # Gemini への問い合わせ回数
        self.last_query_time = -math.inf
        self.blocked = 0               # 障害物で止まった回数
        self.pan = 0.0                 # 今の首振りサーボの角度 [deg]
        self.tilt = None               # 今のうなずきサーボの角度 [deg]

    # --- センサの受信 ---

    def on_odom(self, msg: Odometry):
        self.odom = msg
        self.last_odom_time = time.monotonic()

    def on_scan(self, msg: LaserScan):
        self.scan = msg
        self.last_scan_time = time.monotonic()

    def on_image(self, msg: CompressedImage):
        self.jpeg = bytes(msg.data)
        self.image_time = time.monotonic()

    def fresh_image(self) -> bytes:
        """今の向きで撮られた画像を返す（動き終わってから CAMERA_SETTLE 秒以上後に届いたもの）。"""
        after = time.monotonic() + CAMERA_SETTLE
        end = after + 3.0  # 画像が途絶えていても止まらないよう、待つのは最大 3 秒
        while self.image_time < after and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        return self.jpeg

    def yaw(self) -> float:
        q = self.odom.pose.pose.orientation
        return yaw_from_quaternion(q.z, q.w)

    def position(self) -> tuple[float, float]:
        p = self.odom.pose.pose.position
        return p.x, p.y

    def distance(self, center_deg: float) -> float:
        """LiDAR で、center_deg の方向（正面 0、後方 180）の一番近い物までの距離 [m]。"""
        s = self.scan
        return sector_distance(s.ranges, s.angle_min, s.angle_increment, center_deg,
                               config.FRONT_HALF_WIDTH_DEG, LIDAR_MIN_RANGE, s.range_max)

    # --- 記録（best-effort：失敗してもロボットは止めない） ---

    def event(self, state: str, **data):
        """状態の切り替わりなどを、ログ・トピック・ファイルに残す。"""
        record = {'t': round(time.monotonic() - self.start_time, 2), 'state': state, **data}
        text = json.dumps(record, ensure_ascii=False)
        self.get_logger().info(text)
        self.event_pub.publish(String(data=text))
        try:
            self.recorder.append_event(record)
        except OSError as e:
            self.get_logger().warning(f'記録に失敗しました: {e}')

    def save_image(self, name: str, jpeg: bytes, target: tuple[float, float] | None):
        try:
            self.recorder.save_image(name, jpeg, target)
        except OSError as e:
            self.get_logger().warning(f'画像の保存に失敗しました: {e}')

    # --- 動作の部品 ---

    def spin_for(self, seconds: float):
        """指定時間、受信処理だけを行う（最新のセンサ値に更新する）。"""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_for_sensors(self) -> bool:
        """起動時に、必要なセンサがすべて届くまで待つ。

        Returns:
            すべて届けば True。届かないものがあればその名前をログに出して False。
        """
        end = time.monotonic() + STARTUP_TIMEOUT
        while time.monotonic() < end and None in (self.odom, self.scan, self.jpeg):
            rclpy.spin_once(self, timeout_sec=0.1)
        missing = [name for name, value in
                   (('/odom_raw', self.odom), ('/scan', self.scan), ('camera', self.jpeg))
                   if value is None]
        if missing:
            self.event('startup_failed', missing=missing)
        return not missing

    def check_motion(self, deadline: float):
        """動作中の安全確認。センサが途絶えたか、動作が時間内に終わらなければ SafetyStop。"""
        now = time.monotonic()
        if now - self.last_scan_time > SENSOR_TIMEOUT or now - self.last_odom_time > SENSOR_TIMEOUT:
            raise SafetyStop('センサが途絶えました')
        if now > deadline:
            raise SafetyStop('動作が時間内に終わりませんでした（車輪の空回りなど）')

    def drive(self, linear: float, angular: float):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_pub.publish(msg)

    def turn(self, angle: float):
        """その場で angle [rad] だけ旋回する（左回りが正）。"""
        if abs(angle) < math.radians(config.MIN_TURN_DEG):
            return
        self.camera_for_moving()
        start = self.yaw()
        deadline = time.monotonic() + MOTION_TIMEOUT_FACTOR * abs(angle) / config.TURN_SPEED + 2.0
        while abs(normalize_angle(self.yaw() - start)) < abs(angle):
            self.check_motion(deadline)
            self.drive(0.0, math.copysign(config.TURN_SPEED, angle))
            rclpy.spin_once(self, timeout_sec=1.0 / RATE)
        self.stop()

    def move(self, distance: float) -> bool:
        """distance [m] だけ直進する（負なら後退）。進む向きに障害物があれば手前で止まる。

        Returns:
            障害物が停止距離以内になって途中で止まった場合 True。
        """
        # LiDAR で見る向きと、止まる距離（LiDAR の中心から車体の端まで＋余裕）
        direction, stop = (0, FRONT_EDGE) if distance > 0 else (180, REAR_EDGE)
        stop += config.STOP_MARGIN
        self.camera_for_moving()
        x0, y0 = self.position()
        deadline = time.monotonic() + MOTION_TIMEOUT_FACTOR * abs(distance) / config.FORWARD_SPEED + 2.0
        while math.dist((x0, y0), self.position()) < abs(distance):
            self.check_motion(deadline)
            if self.distance(direction) <= stop:
                self.stop()
                return True
            self.drive(math.copysign(config.FORWARD_SPEED, distance), 0.0)
            rclpy.spin_once(self, timeout_sec=1.0 / RATE)
        self.stop()
        return False

    def stop(self):
        """ロボットが止まったことをオドメトリで確認できるまで停止指令を送る。"""
        deadline = time.monotonic() + STOP_TIMEOUT
        while time.monotonic() < deadline:
            self.drive(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=1.0 / RATE)
            t = self.odom.twist.twist
            if abs(t.linear.x) < STOP_TOLERANCE and abs(t.angular.z) < STOP_TOLERANCE:
                return
        self.get_logger().warning(f'{STOP_TIMEOUT} 秒以内に停止を確認できませんでした')

    def set_camera(self, pan_deg: float, tilt_deg: float = config.CAMERA_TILT_LOOK_DEG):
        """カメラを首振り pan_deg、うなずき tilt_deg に向ける（初期値は見るときの角度）。"""
        # 1通では届かないことがあるため繰り返す（角度指定なので何回送っても結果は同じ）
        for _ in range(3):
            self.servo_pubs[0].publish(Int32(data=int(pan_deg)))
            self.servo_pubs[1].publish(Int32(data=int(tilt_deg)))
            time.sleep(0.05)
        self.pan, self.tilt = pan_deg, tilt_deg
        self.spin_for(0.5)  # サーボが動き終わり、新しい画像・LiDAR の値が届くのを待つ

    def camera_for_moving(self):
        """動く前に、カメラを LiDAR の測定面から外す（下に向ける）。"""
        # カメラの頭が LiDAR の測定面に入ると、正面を自分のカメラで塞いでしまう
        if (self.pan, self.tilt) != (0, config.CAMERA_TILT_MOVE_DEG):
            self.set_camera(0, config.CAMERA_TILT_MOVE_DEG)
            # 下げる途中でカメラが LiDAR の測定面を横切るため、外れた後の新しい値が届くまで待つ
            self.spin_for(0.5)

    def beep(self, ms: int):
        # 1通では届かないことがあるため、ブザーの指令も念のため繰り返す
        for _ in range(3):
            self.beep_pub.publish(UInt16(data=ms))
            time.sleep(0.05)

    # --- AI への問い合わせ ---

    def ask_gemini(self, jpegs: list[bytes], prompt: str) -> str | None:
        """画像（複数可）とプロンプトを Gemini に送り、答えの文章を得る。

        API エラーのときは、待ってから聞き直す（問い合わせ上限まで）。

        Returns:
            Gemini の答え。問い合わせ上限に達した場合は None。
        """
        while self.queries < config.MAX_QUERIES:
            self.queries += 1
            # 回数制限にかからないよう、前回から間隔をあける（有料プランなら 0）
            self.spin_for(self.last_query_time + config.QUERY_INTERVAL - time.monotonic())
            self.last_query_time = time.monotonic()
            parts = [types.Part.from_bytes(data=j, mime_type='image/jpeg') for j in jpegs]
            try:
                res = self.client.models.generate_content(
                    model=MODEL,
                    contents=[*parts, prompt],
                    config=types.GenerateContentConfig(
                        thinking_config=types.ThinkingConfig(thinking_level=config.THINKING_LEVEL)),
                )
                return res.text or ''
            except errors.APIError as e:
                self.event('api_error', query=self.queries, error=str(e)[:200])
                self.spin_for(API_RETRY_WAIT)
        return None

    def look(self, tilt_deg: float = config.CAMERA_TILT_LOOK_DEG) -> tuple[float, float, float] | None:
        """正面のカメラ画像1枚で目標を探す。

        Args:
            tilt_deg: カメラのうなずき角 [deg]。足元を確かめるときは下に向ける。

        Returns:
            (目標の方向, 目標の縦位置, 撮影時のうなずき角)。方向は車体を向けるための旋回角度 [rad]
            （左回りが正）、縦位置は画像の上端 0・下端 1。見つからなければ None。
        """
        self.set_camera(0, tilt_deg)
        jpeg = self.fresh_image()
        answer = self.ask_gemini([jpeg], build_prompt(self.target))
        target = None if answer is None else parse_target(answer)
        self.save_image(f'{self.queries:03d}', jpeg, target)
        self.event('look', query=self.queries, tilt_deg=tilt_deg, answer=answer, target=target,
                   latency=round(time.monotonic() - self.last_query_time, 2))
        self.spin_for(0.3)  # 問い合わせ中に止まっていた受信処理を進め、センサ値を更新する
        if target is None:
            return None
        return target_bearing(0, target[0], config.CAMERA_HFOV_DEG), target[1], tilt_deg

    def sweep(self, kind: str, turn_deg: float, pans: list, tilts: list) -> tuple[float, float, float] | None:
        """車体を一周させながらカメラを振って画像を撮りため、まとめて1回で目標を探す。

        Args:
            kind: 記録用の名前（"sweep"：粗い一周撮影、"detail"：詳細探索）。
            turn_deg: 撮影のたびに車体を回す角度 [deg]。
            pans: 各向きでの首振り角 [deg] のリスト。
            tilts: 各向きでのうなずき角 [deg] のリスト。

        Returns:
            look() と同じ。見つからなければ None。
        """
        captures = []  # (画像, 撮影時の車体の向き, 首振り角, うなずき角)
        turns = round(360 / turn_deg)
        for i in range(turns):
            for tilt in tilts:
                for pan in pans:
                    self.set_camera(pan, tilt)
                    captures.append((self.fresh_image(), self.yaw(), pan, tilt))
            if i < turns - 1:
                self.turn(math.radians(turn_deg))
        self.set_camera(0)

        answer = self.ask_gemini([c[0] for c in captures],
                                 build_sweep_prompt(self.target, len(captures)))
        found = None if answer is None else parse_sweep(answer)
        for k, (jpeg, _, _, _) in enumerate(captures):
            hit = found is not None and found[0] == k
            self.save_image(f'{self.queries:03d}_{k:02d}', jpeg, found[1:] if hit else None)
        self.event(kind, query=self.queries, images=len(captures), answer=answer, found=found,
                   latency=round(time.monotonic() - self.last_query_time, 2))
        self.spin_for(0.3)
        if found is None or not 0 <= found[0] < len(captures):
            return None
        _, capture_yaw, pan, tilt = captures[found[0]]
        bearing = heading_to_capture(capture_yaw, self.yaw(), pan, found[1], config.CAMERA_HFOV_DEG)
        return bearing, found[2], tilt

    # --- 全体の手順 ---

    def explore(self, distance: float) -> bool:
        """目標が見つからないときに、十分に空いている方向のどれかへ少し移動して、探す場所を変える。

        Args:
            distance: 進む距離 [m]。

        Returns:
            続けてよい場合 True。障害物で止まった回数が上限を超えたら False。
        """
        self.camera_for_moving()  # LiDAR の視界からカメラを外してから方向を選ぶ
        s = self.scan
        candidates = open_directions(s.ranges, s.angle_min, s.angle_increment, 30,
                                     config.FRONT_HALF_WIDTH_DEG, LIDAR_MIN_RANGE, s.range_max,
                                     config.EXPLORE_OPEN_RATIO)
        # 毎回同じ方向を往復しないよう、候補からランダムに選ぶ
        direction = random.choice(candidates)
        self.event('explore', direction_deg=direction, distance=distance)
        self.turn(math.radians(direction))
        if self.move(distance):
            return self.back_off(adjust=True)
        return True

    def open_direction(self) -> float:
        """LiDAR で一番遠くまで空いている方向 [deg]（正面 0、左回りが正）。"""
        s = self.scan
        return most_open_direction(s.ranges, s.angle_min, s.angle_increment, 30,
                                   config.FRONT_HALF_WIDTH_DEG, LIDAR_MIN_RANGE, s.range_max)

    def back_off(self, adjust: bool) -> bool:
        """前方の障害物で止まったときに、少し下がる。

        Args:
            adjust: 下がった後、LiDAR で一番空いている方向へ向き直すか。
                目標に向かっている途中なら、目標を見失わないよう False にする。

        Returns:
            やり直す場合 True。障害物で続けて止まった回数が上限を超えたら False。
        """
        self.blocked += 1
        self.event('blocked', count=self.blocked, front=round(self.distance(0), 2))
        if self.blocked > config.MAX_BLOCKED:
            self.event('give_up', reason='blocked')
            return False
        self.event('back', distance=config.BACKUP_DISTANCE)
        self.move(-config.BACKUP_DISTANCE)  # 後方に物があれば、その手前で止まる
        if not adjust:
            return True
        # 同じ障害物に何度も当たらないよう、空いている方向へ向き直す
        direction = self.open_direction()
        self.event('adjust', direction_deg=direction)
        self.turn(math.radians(direction))
        return True

    def run(self):
        """探す → 近づく → 見失ったら詳細に探す を繰り返し、到着・上限・安全停止のいずれかで終える。

        - 探す：正面で見る → 見つからなければ粗く一周撮影 → それでもなければ空いている方向へ移動
        - 近づく：目標の方向へ旋回して前進
        - 見失った：詳細探索（細かい角度・上下にも振る）→ なければ少しだけ動いて再度詳細探索。
          続けて MAX_DETAIL_SEARCHES 回見つからなければ「探す」に戻る
        """
        if not self.wait_for_sensors():
            return
        self.set_camera(0)
        self.event('start', target=self.target)
        approaching = False  # 直前に目標へ向かって前進したか
        tracking = False     # 目標を一度見つけて追っている状態か（見失ったら詳細探索に入る）
        details = 0          # 続けて行った詳細探索の回数
        while True:
            if time.monotonic() - self.start_time > config.TIME_LIMIT:
                self.event('give_up', reason='time_limit')
                return
            if self.queries >= config.MAX_QUERIES:
                self.event('give_up', reason='max_queries')
                return
            self.camera_for_moving()  # 前方の確認も、カメラを外した状態で行う
            if self.distance(0) <= FRONT_EDGE + config.STOP_MARGIN:
                if not self.back_off(adjust=True):
                    return
                continue

            found = self.look()
            # 近づいた直後に見失ったら、視野の下に外れた可能性が高いので足元を確かめる
            if found is None and approaching:
                if self.look(config.CAMERA_TILT_NEAR_DEG) is not None:
                    self.event('arrived', reason='at_feet')
                    self.beep(500)
                    return
                tracking = True
            approaching = False

            if found is None and tracking:
                if details >= config.MAX_DETAIL_SEARCHES:
                    # 同じ付近を探し続けないよう、通常の探索に戻る
                    self.event('back_to_search', details=details)
                    tracking, details = False, 0
                    continue
                details += 1
                found = self.sweep('detail', config.DETAIL_TURN_DEG,
                                   config.DETAIL_PANS_DEG, config.DETAIL_TILTS_DEG)
                if found is None:
                    if not self.explore(config.DETAIL_MOVE_DISTANCE):
                        return
                    continue
            elif found is None:
                found = self.sweep('sweep', config.SWEEP_TURN_DEG,
                                   config.SWEEP_PANS_DEG, config.SWEEP_TILTS_DEG)
                if found is None:
                    if not self.explore(config.EXPLORE_DISTANCE):
                        return
                    continue
            # 目標が見えた（以後、見失ったら詳細探索に入る）
            tracking, details = True, 0

            bearing, y, tilt = found
            # 通常の角度より下向きの画像で見つかった＝足元近くにある
            near = tilt < config.CAMERA_TILT_LOOK_DEG or y >= config.NEAR_Y
            self.event('turn', angle_deg=round(math.degrees(bearing), 1))
            self.turn(bearing)
            # カメラは下向きなので、目標が画像の下のほうに見えたら目の前にある
            if tilt == config.CAMERA_TILT_LOOK_DEG and y >= config.ARRIVE_Y:
                self.event('arrived', target_y=y)
                self.beep(500)
                return
            # 大きく回ったときは旋回の誤差が大きいので、進む前に見直して向きを確かめる
            if abs(bearing) > math.radians(config.ALIGN_TOLERANCE_DEG):
                continue
            # 近いときは、視野から外れないよう小刻みに進む
            step = config.STEP_DISTANCE_NEAR if near else config.STEP_DISTANCE
            self.camera_for_moving()
            self.event('forward', distance=step, front=round(self.distance(0), 2))
            approaching = True
            if self.move(step):
                # 目標の方を向いたまま下がり、見直す（向き直すと目標を見失う）
                if not self.back_off(adjust=False):
                    return
            else:
                self.blocked = 0  # 障害物なく進めたら、やり直しの回数を数え直す


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('target', help='目標を表す言葉（例: "pink tape"）')
    args = parser.parse_args(remove_ros_args()[1:])

    # Humble 標準の Ctrl+C 処理は通信を先に閉じて停止指令を送れなくするため無効にする（guide/04）
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = GoTo(args.target)
    try:
        node.run()
    except KeyboardInterrupt:
        node.event('interrupted')
    except SafetyStop as e:
        node.event('safety_stop', reason=str(e))
    finally:
        # どう終わっても、必ず止めてから終わる
        if node.odom is not None:
            node.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
