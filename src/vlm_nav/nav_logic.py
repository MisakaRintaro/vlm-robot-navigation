"""ROS や Gemini に依存しない、移動の判断に使う計算。"""
import json
import math
import re


def build_prompt(target: str) -> str:
    """目標の位置を Gemini Robotics-ER に答えさせるプロンプトを作る。

    Args:
        target: 目標を表す言葉（例："pink tape"）。

    Returns:
        画像と一緒に送るプロンプト。答えは [y, x]（0〜1000）の JSON、見えなければ []。
    """
    return (
        f'Point to the {target} in the image. '
        'Answer in JSON: [{"point": [y, x], "label": <name>}]. '
        'Points are [y, x] normalized to 0-1000. '
        'If it is not visible, answer [].'
    )


def build_sweep_prompt(target: str, count: int) -> str:
    """複数の画像から目標を探させるプロンプトを作る。

    Args:
        target: 目標を表す言葉。
        count: 一緒に送る画像の枚数。画像は送った順に 0 から番号が付く。

    Returns:
        画像と一緒に送るプロンプト。答えは {"image": 番号, "point": [y, x]}、見つからなければ {}。
    """
    return (
        f'You are given {count} images, numbered 0 to {count - 1} in order. '
        f'Find the {target}. '
        'Answer in JSON: {"image": <number>, "point": [y, x]} for the image where it is most clearly visible. '
        'Points are [y, x] normalized to 0-1000. '
        'If it is not visible in any image, answer {}.'
    )


def parse_sweep(text: str) -> tuple[int, float, float] | None:
    """複数画像の探索に対する Gemini の答えから、どの画像のどこにあるかを取り出す。

    Args:
        text: Gemini の答え。```json で囲まれていてもよい。

    Returns:
        (画像の番号, x, y)。x, y は画像の左上 (0, 0)・右下 (1, 1)。見つからない・読み取れない場合は None。
    """
    match = re.search(r'\{.*\}', text, re.DOTALL)
    if match is None:
        return None
    try:
        answer = json.loads(match.group(0))
        y, x = answer['point']
        return int(answer['image']), x / 1000, y / 1000
    except (ValueError, LookupError, TypeError):
        return None


def parse_target(text: str) -> tuple[float, float] | None:
    """Gemini の答えから、目標の画像上の位置を取り出す。

    Args:
        text: Gemini の答え。```json で囲まれていてもよい。

    Returns:
        (x, y)。画像の左上を (0, 0)、右下を (1, 1) とする。
        見えない（[]）または読み取れない場合は None。
    """
    # 答えの中の最初の JSON 配列だけを読む（```json などの装飾を無視する）
    match = re.search(r'\[.*\]', text, re.DOTALL)
    if match is None:
        return None
    try:
        points = json.loads(match.group(0))
        y, x = points[0]['point']
    except (ValueError, LookupError, TypeError):
        return None
    return x / 1000, y / 1000


def heading_error(x: float, hfov_deg: float) -> float:
    """画像の横位置から、目標を正面にするための旋回角度を求める。

    Args:
        x: 目標の横位置（左端 0、右端 1）。
        hfov_deg: カメラの水平視野角 [deg]。

    Returns:
        旋回角度 [rad]。REP 103 に従い、左回りが正。
    """
    return math.radians((0.5 - x) * hfov_deg)


def target_bearing(pan_deg: float, x: float, hfov_deg: float) -> float:
    """首振りの角度と画像の横位置から、目標の方向（車体の正面から見た角度）を求める。

    Args:
        pan_deg: 撮影したときの首振りサーボ S1 の角度 [deg]。実機では右が正。
        x: 目標の横位置（左端 0、右端 1）。
        hfov_deg: カメラの水平視野角 [deg]。

    Returns:
        車体を目標に向けるための旋回角度 [rad]。REP 103 に従い、左回りが正。
    """
    # サーボは右が正、ROS の角度は左が正なので符号を反転する
    return math.radians(-pan_deg) + heading_error(x, hfov_deg)


def heading_to_capture(capture_yaw: float, current_yaw: float, pan_deg: float,
                       x: float, hfov_deg: float) -> float:
    """以前に撮った画像に写っていた目標へ、今の向きから回る角度を求める。

    Args:
        capture_yaw: 撮影したときの車体の向き [rad]（オドメトリ）。
        current_yaw: 今の車体の向き [rad]。
        pan_deg: 撮影したときの首振りサーボ S1 の角度 [deg]。右が正。
        x: その画像での目標の横位置（左端 0、右端 1）。
        hfov_deg: カメラの水平視野角 [deg]。

    Returns:
        旋回角度 [rad]。左回りが正。回る量が少ない向き（-π〜π）を選ぶ。
    """
    return normalize_angle(capture_yaw + target_bearing(pan_deg, x, hfov_deg) - current_yaw)


def sector_distance(ranges, angle_min: float, angle_increment: float, center_deg: float,
                    half_width_deg: float, range_min: float, range_max: float) -> float:
    """LiDAR のデータから、指定した方向の範囲で一番近い物までの距離を求める。

    Args:
        ranges: LaserScan.ranges。
        angle_min: LaserScan.angle_min [rad]。
        angle_increment: LaserScan.angle_increment [rad]。
        center_deg: 範囲の中心の方向 [deg]。正面 0、後方 180（左回りが正）。
        half_width_deg: 中心から左右何度までを範囲とするか [deg]。
        range_min: これ以下の値は無効として捨てる [m]。
        range_max: これ以上の値は無効として捨てる [m]。

    Returns:
        範囲内で 2 番目に近い距離 [m]。有効な値が 2 点未満なら inf。

    Note:
        実機の LiDAR は 1 点だけ近い値（ノイズ）を出すことがあるため、1 点だけの外れ値は無視する。
    """
    center = math.radians(center_deg)
    half_width = math.radians(half_width_deg)
    valid = []
    for i, r in enumerate(ranges):
        # 後方のように ±180° をまたぐ範囲も扱えるよう、中心との差を -π〜π に直して比べる
        offset = normalize_angle(angle_min + i * angle_increment - center)
        if abs(offset) <= half_width and range_min < r < range_max:
            valid.append(r)
    if len(valid) < 2:
        return math.inf
    return sorted(valid)[1]


def yaw_from_quaternion(z: float, w: float) -> float:
    """平面上を動くロボットの姿勢（クォータニオン）から向きを求める。

    Args:
        z: orientation.z。
        w: orientation.w。

    Returns:
        向き [rad]。左回りが正。
    """
    return 2 * math.atan2(z, w)


def normalize_angle(angle: float) -> float:
    """角度を -π〜π の範囲に直す。

    Args:
        angle: 角度 [rad]。

    Returns:
        同じ向きを表す -π〜π の角度 [rad]。向きの差を求めるときに使う（179° → -179° を +2° とする）。
    """
    return math.atan2(math.sin(angle), math.cos(angle))


def most_open_direction(ranges, angle_min: float, angle_increment: float, step_deg: float,
                        half_width_deg: float, range_min: float, range_max: float) -> float:
    """LiDAR のデータから、一番遠くまで空いている方向を選ぶ。

    Args:
        ranges: LaserScan.ranges。
        angle_min: LaserScan.angle_min [rad]。
        angle_increment: LaserScan.angle_increment [rad]。
        step_deg: 候補の方向の間隔 [deg]（例：30 なら -180, -150, …, 150）。
        half_width_deg: 各方向の範囲の半分の幅 [deg]。
        range_min: これ以下の値は無効として捨てる [m]。
        range_max: これ以上の値は無効として捨てる [m]。

    Returns:
        方向 [deg]（正面 0、左回りが正）。有効な値がある方向がなければ 0。

    Note:
        有効な値がない方向（何も返ってこない方向）は、空いているとは限らないので選ばない。
    """
    best_deg, best_dist = 0, 0.0
    for deg in range(-180, 180, int(step_deg)):
        dist = sector_distance(ranges, angle_min, angle_increment, deg,
                               half_width_deg, range_min, range_max)
        if dist != math.inf and dist > best_dist:
            best_deg, best_dist = deg, dist
    return best_deg


def open_directions(ranges, angle_min: float, angle_increment: float, step_deg: float,
                    half_width_deg: float, range_min: float, range_max: float,
                    ratio: float) -> list[float]:
    """LiDAR のデータから、十分に空いている方向の候補を挙げる。

    Args:
        ranges, angle_min, angle_increment, step_deg, half_width_deg, range_min, range_max:
            most_open_direction() と同じ。
        ratio: 一番空いている方向の距離に対して、この割合以上空いていれば候補にする（0〜1）。

    Returns:
        候補の方向 [deg] のリスト（正面 0、左回りが正）。有効な値がある方向がなければ [0]。

    Note:
        候補からランダムに選ぶと、毎回同じ方向を往復するのを避けられる。
    """
    dists = {}
    for deg in range(-180, 180, int(step_deg)):
        dist = sector_distance(ranges, angle_min, angle_increment, deg,
                               half_width_deg, range_min, range_max)
        if dist != math.inf:
            dists[deg] = dist
    if not dists:
        return [0]
    best = max(dists.values())
    return [deg for deg, dist in dists.items() if dist >= ratio * best]
