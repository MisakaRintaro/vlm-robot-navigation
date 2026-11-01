from battery_monitor import to_volts


def test_to_volts_converts_raw_value_to_volts():
    # 公式資料の例：data=82 は 8.2 V
    assert to_volts(82) == 8.2


def test_to_volts_zero():
    assert to_volts(0) == 0.0
