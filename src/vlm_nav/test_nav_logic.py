import math

import pytest

from nav_logic import (
    build_prompt,
    build_sweep_prompt,
    heading_error,
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


# --- Gemini の答えの読み取り ---

def test_parse_target_returns_normalized_x_y():
    assert parse_target('[{"point": [348, 758], "label": "tape"}]') == (0.758, 0.348)


def test_parse_target_accepts_markdown_code_fence():
    # 実際の Gemini の答えは ```json で囲まれていた
    text = '```json\n[\n  {"point": [348, 758], "label": "tape"}\n]\n```'
    assert parse_target(text) == (0.758, 0.348)


def test_parse_target_returns_none_when_not_visible():
    assert parse_target('[]') is None


def test_parse_target_returns_none_for_unreadable_answer():
    assert parse_target('I cannot see it.') is None


# --- 画像の横位置から旋回角度 ---

def test_heading_error_is_zero_at_image_center():
    assert heading_error(0.5, hfov_deg=60) == 0.0


def test_heading_error_is_positive_for_target_on_left():
    # REP 103：左回り（反時計回り）が正
    assert heading_error(0.0, hfov_deg=60) == pytest.approx(math.radians(30))


def test_heading_error_is_negative_for_target_on_right():
    assert heading_error(1.0, hfov_deg=60) == pytest.approx(math.radians(-30))


def test_target_bearing_adds_camera_pan():
    # 首を右に 60° 振った画像の中央に目標 → 車体は右に 60°（右回りは負）
    assert target_bearing(pan_deg=60, x=0.5, hfov_deg=60) == pytest.approx(math.radians(-60))


def test_target_bearing_with_center_pan_equals_heading_error():
    assert target_bearing(pan_deg=0, x=0.0, hfov_deg=60) == pytest.approx(math.radians(30))


# --- LiDAR の方向別の距離 ---

def make_ranges(default=2.0):
    # 実機と同じ：-180°〜+179°、1° 刻み、360 点
    return [default] * 360


ANGLE_MIN = math.radians(-180)
INC = math.radians(1)


def sector(ranges, center_deg):
    return sector_distance(ranges, ANGLE_MIN, INC, center_deg=center_deg, half_width_deg=15,
                           range_min=0.12, range_max=8.0)


def test_sector_distance_returns_nearest_within_front_sector():
    ranges = make_ranges()
    ranges[180 + 10] = ranges[180 + 11] = 0.4   # +10°, +11°（前方 ±15° の内側）
    ranges[180 + 40] = ranges[180 + 41] = 0.2   # +40°, +41°（外側なので無視）
    assert sector(ranges, center_deg=0) == 0.4


def test_sector_distance_ignores_single_outlier():
    # 実機では 1 点だけ近い値（ノイズ）が出ることがある。2 点以上近いときだけ物があるとみなす
    ranges = make_ranges()
    ranges[180 + 5] = 0.13
    assert sector(ranges, center_deg=0) == 2.0


def test_sector_distance_handles_rear_across_180_degrees():
    # 後方（180°）の範囲は -180° と +179° の両側にまたがる
    ranges = make_ranges()
    ranges[0 + 5] = 0.3      # -175°
    ranges[359] = 0.3        # +179°
    assert sector(ranges, center_deg=180) == 0.3


def test_sector_distance_ignores_invalid_values():
    # 実機で 0.00 などの無効値が混じる
    ranges = make_ranges()
    ranges[180] = 0.0
    assert sector(ranges, center_deg=0) == 2.0


def test_sector_distance_is_infinite_when_nothing_valid():
    assert sector(make_ranges(default=0.0), center_deg=0) == math.inf


def open_dir(ranges):
    return most_open_direction(ranges, ANGLE_MIN, INC, step_deg=30, half_width_deg=15,
                               range_min=0.12, range_max=8.0)


def test_most_open_direction_picks_farthest_sector():
    ranges = make_ranges(default=0.5)
    for deg in range(70, 111):       # 左 90° の方向（±15° より広く）だけ遠くまで空いている
        ranges[180 + deg] = 3.0
    assert open_dir(ranges) == 90


def test_most_open_direction_ignores_sectors_without_valid_values():
    # 有効な値がない方向（机の外など、何も返ってこない方向）は選ばない
    ranges = make_ranges(default=0.5)
    for deg in range(-110, -69):
        ranges[180 + deg] = 0.0
    for deg in range(10, 51):
        ranges[180 + deg] = 1.0
    assert open_dir(ranges) == 30


def test_most_open_direction_defaults_to_front_when_nothing_valid():
    assert open_dir(make_ranges(default=0.0)) == 0


def test_open_directions_lists_directions_at_least_as_open_as_ratio_of_best():
    ranges = make_ranges(default=0.5)
    for deg in range(70, 111):      # 左 90° は 3.0 m
        ranges[180 + deg] = 3.0
    for deg in range(-110, -69):    # 右 90° は 2.0 m（最大の 6 割以上）
        ranges[180 + deg] = 2.0
    dirs = open_directions(ranges, ANGLE_MIN, INC, step_deg=30, half_width_deg=15,
                           range_min=0.12, range_max=8.0, ratio=0.6)
    assert sorted(dirs) == [-90, 90]


# --- オドメトリの向き ---

def test_yaw_from_quaternion_for_90_degrees_left():
    s = math.sin(math.radians(45))
    assert yaw_from_quaternion(z=s, w=s) == pytest.approx(math.radians(90))


def test_normalize_angle_wraps_across_180_degrees():
    # 179° から左に 2° 回ると -179° になる。差は +2° として扱う
    diff = normalize_angle(math.radians(-179) - math.radians(179))
    assert diff == pytest.approx(math.radians(2))


# --- まとめて送る探索（スイープ） ---

def test_parse_sweep_returns_image_index_and_position():
    text = '```json\n{"image": 3, "point": [500, 250]}\n```'
    assert parse_sweep(text) == (3, 0.25, 0.5)


def test_parse_sweep_returns_none_when_not_found():
    assert parse_sweep('{}') is None


def test_parse_sweep_returns_none_for_unreadable_answer():
    assert parse_sweep('not found') is None


def test_heading_to_capture_accounts_for_rotation_since_capture():
    # 撮影時は今より左に 90° 向いていた。その画像の中央・首は正面 → 左に 90° 回る
    turn = heading_to_capture(capture_yaw=math.radians(90), current_yaw=0.0,
                              pan_deg=0, x=0.5, hfov_deg=60)
    assert turn == pytest.approx(math.radians(90))


def test_heading_to_capture_wraps_to_shortest_turn():
    # 撮影時の向き 170°、今 -170° → 左に 340° ではなく右に 20° 回る
    turn = heading_to_capture(capture_yaw=math.radians(170), current_yaw=math.radians(-170),
                              pan_deg=0, x=0.5, hfov_deg=60)
    assert turn == pytest.approx(math.radians(-20))


# --- プロンプト ---

def test_build_prompt_contains_target():
    assert 'pink tape' in build_prompt('pink tape')


def test_build_sweep_prompt_contains_target_and_image_count():
    prompt = build_sweep_prompt('pink tape', 12)
    assert 'pink tape' in prompt and '12' in prompt
