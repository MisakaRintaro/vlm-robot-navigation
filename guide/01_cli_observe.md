# 01. CLI で観る

コードは書かず、`ros2` コマンドでロボットから届くデータを観察する。

## ゴール
- トピックには **型** があり、**中身** と **周期** を観察できると理解する

## トピック一覧
| トピック | 向き | 型 | 中身 |
|---|---|---|---|
| `/battery` | ロボット → PC | `std_msgs/UInt16` | 電池電圧。値÷10 が [V] |
| `/imu` | ロボット → PC | `sensor_msgs/Imu` | 加速度・角速度（約25Hz） |
| `/odom_raw` | ロボット → PC | `nav_msgs/Odometry` | 車輪から計算した位置・速度 |
| `/scan` | ロボット → PC | `sensor_msgs/LaserScan` | LiDAR の距離（360°） |
| `/espRos/esp32camera` | カメラ → PC | `sensor_msgs/CompressedImage` | JPEG 画像 |
| `/cmd_vel` | PC → ロボット | `geometry_msgs/Twist` | 走行指令 |
| `/beep` | PC → ロボット | `std_msgs/UInt16` | 0=停止、1=鳴り続ける、10以上=その ms だけ鳴る |
| `/servo_s1` / `/servo_s2` | PC → ロボット | `std_msgs/Int32` | カメラの首振り。s1 は [-90, 90]、s2 は [-90, 20] |

## やること
`ros` コンテナのシェルで1つずつ実行する（`Ctrl+C` で停止）。

| # | コマンド | 確認すること |
|---|---|---|
| ① | `ros2 topic type /battery` | `std_msgs/msg/UInt16` と出る |
| ② | `ros2 topic echo /battery --once` | `data: 81` など → ÷10 で 8.1V |
| ③ | `ros2 topic hz /imu` | 約25Hz |
| ④ | `ros2 topic echo /imu --field linear_acceleration` | 置いたままなら z ≒ 9.8（重力）。手で傾けると x/y/z の値が入れ替わる |

## 結果メモ
- ② 8.1V（2026-10-31）
- ③ 約25Hz（間隔 0.012〜0.065s）。周期にはばらつきがある（WiFi 経由のため）。Hz は「1秒に何回届くか」
- ④ 水平に置いた状態で x≒0.05、y≒0.04、z≒9.81 → 重力がそのまま z（上向き）に出ている

## 型定義の読み方（`ros2 interface show`）
```
std_msgs/Header header              ← 「型 名前」。header は Header 型のかたまり
        builtin_interfaces/Time stamp   ← 字下げ＝内側の要素
                int32 sec
                uint32 nanosec
        string frame_id
float64[9] orientation_covariance   ← [9] は 9 個の配列
```
- `#` の行はコメント。単位などの決まりが書いてある（Imu なら加速度 m/s²、角速度 rad/s）
- 入れ子はドットでたどる。**`--field` のパスと Python のアクセスは同じ**

| CLI | Python |
|---|---|
| `ros2 topic echo /imu --field header.stamp.sec` | `msg.header.stamp.sec` |
| `ros2 topic echo /imu --field linear_acceleration.z` | `msg.linear_acceleration.z` |

- `header` はセンサ系メッセージ共通の「荷札」。`stamp`（測定時刻）は複数センサの時刻合わせや rosbag に、`frame_id`（座標系名）は rviz2 での表示位置に使う

## このロボットの /imu の実際
| フィールド | 値 | 解釈 |
|---|---|---|
| `orientation` | (0, 0, 0, 1) 固定 | 未計算（初期値のまま）。使えない |
| `angular_velocity` | 静止時 0 | 使える。水平に回すと z が動く |
| `linear_acceleration` | 静止時 z≒9.81 | 使える |
| `*_covariance` | 全部 0 | 信頼度は不明 |

型の定義だけでは「実際に値が入っているか」はわからない。**echo して確かめる**。
