from drive_forward import is_stopped, velocity_at


def test_moves_at_speed_before_duration():
    assert velocity_at(elapsed=0.5, duration=1.0, speed=0.1) == 0.1


def test_stops_after_duration():
    assert velocity_at(elapsed=1.0, duration=1.0, speed=0.1) == 0.0


def test_is_stopped_when_speeds_are_nearly_zero():
    assert is_stopped(linear_x=0.003, angular_z=-0.002)


def test_is_not_stopped_while_moving_forward():
    # 実測の停止直後の値（0.018 m/s）はまだ動いているとみなす
    assert not is_stopped(linear_x=0.018, angular_z=0.0)


def test_is_not_stopped_while_turning():
    assert not is_stopped(linear_x=0.0, angular_z=0.5)
