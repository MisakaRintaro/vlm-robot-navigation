# 言葉で指示できるロボット（Physical AI 応用編 最終課題）

「ピンクのテープのところへ行って」のように言葉で指示すると、ロボットがカメラで目標を探し、近づいて止まる。

- **頭（AI）**：ロボット向け VLM **Gemini Robotics-ER 2** が、カメラ画像と指示から「目標が画像のどこにあるか」を答える
- **体（制御）**：オドメトリで旋回・前進し、LiDAR で障害物の手前で止まる。ROS 2 Humble（WSL2 + Docker）で動かす
- デモ動画：（YouTube の URL を記入）
- 設計：[docs/design.md](docs/design.md)

使用ロボット：Yahboom MicroROS-ESP32 ロボットカー（カメラ・LiDAR 付き）

## 構成
```
ロボット(ESP32) ──WiFi/UDP 8090──▶ [agent]        ─┐
カメラ          ──WiFi/UDP 9999──▶ [camera-agent] ─┼─ ROS 2 (DDS) ─▶ [ros] 自作ノード・ros2 コマンド
                                                    ┘
```
- ESP32 は軽量版の **micro-ROS** で通信する。**micro-ROS Agent** がそれを通常の ROS 2 に変換する
- 3つのコンテナは `network_mode: host` と `ROS_DOMAIN_ID=20` を共有しているので、自動で互いを見つける

| コンテナ | 役割 |
|---|---|
| `agent` | ロボット本体用の micro-ROS Agent（UDP 8090） |
| `camera-agent` | カメラ用の micro-ROS Agent（UDP 9999） |
| `ros` | 作業用。`ros2` コマンドや自作ノードを実行する |

| パス | 内容 |
|---|---|
| [compose.yaml](compose.yaml) | 3つのコンテナの定義 |
| [docker/Dockerfile](docker/Dockerfile) | 作業用コンテナ（ROS 2 Humble + teleop_twist_keyboard） |
| [src/](src/) | 自作 ROS 2 パッケージ。`ros` コンテナの `/ws/src` にマウントされる |
| [docs/design.md](docs/design.md) | 課題「言葉で指示できるロボット」の設計書 |
| [guide/](guide/) | 実機の基礎を学んだときの学習メモ |

## 動作環境
- Windows 11 + WSL2（Ubuntu 24.04、`networkingMode=mirrored`）
- Docker Engine
- ロボット：Yahboom MicroROS-ESP32（ファームウェア V1.1.3）、MS200 LiDAR、ROS-WiFi カメラ
- ロボットとカメラには、PC の IP・UDP ポート・`ROS_DOMAIN_ID=20` を設定しておく（Yahboom 配布の設定ツールを使う。WiFi のパスワードを含むため、このリポジトリには含めない）
- Gemini API キー（[Google AI Studio](https://aistudio.google.com/apikey) で取得）
- PC の IP はルーターの DHCP 固定割当で固定する（変わるとロボットから届かなくなる）

## 使い方
```bash
docker compose up -d                                  # 1. コンテナを起動
# 2. ロボットの電源を入れ、30秒ほど待ってからリセットボタンを押す（Agent の起動「後」に押す）
docker compose logs --since 1m agent | grep session   # 3. "session established" が出れば接続 OK
docker compose exec ros bash                          # 4. 作業シェルに入る
ros2 topic list                                       # 5. /battery /imu /scan などが見えれば OK
```
終了するときは `docker compose down`。

## 言葉で指示して動かす
```bash
cd /ws/src/vlm_nav            # ros コンテナの中
python3 go_to.py "pink tape"  # 危なければ Ctrl+C（停止してから終了する）
```
- 事前に、Gemini の API キーをリポジトリ直下の `.env` に `GEMINI_API_KEY=...` と書いておく（Git には含めない）
- 調整するパラメータは [src/vlm_nav/config.py](src/vlm_nav/config.py)、設計は [docs/design.md](docs/design.md)

### 実行記録の見方
実行するたびに `runs/<日時>/` ができる。
| ファイル | 中身 |
|---|---|
| `events.jsonl` | 経過時間ごとの出来事（AI の答え・応答時間・旋回角・止まった理由）。1行1件の JSON |
| `001.jpg` … | 各問い合わせで AI に送った画像 |
| `NNN_target.jpg` | AI が目標を見つけた画像。指した位置に赤丸 |

センサと一緒に時系列で見たいときは、実行中に別のターミナルで rosbag を記録する。
```bash
ros2 bag record -o /ws/runs/bag_$(date +%Y%m%d_%H%M%S) /cmd_vel /odom_raw /scan /espRos/esp32camera /go_to/event
```

## テスト
```bash
cd /ws/src/vlm_nav && python3 -m pytest -q   # ros コンテナの中
```

## ロボットのトピック
ロボット本体はノード `/YB_Car_Node` として、以下のトピックを publish / subscribe する。

### ロボット → PC（publish）
| トピック | 型 | 周期 | 役割 |
|---|---|---|---|
| `/odom_raw` | `nav_msgs/msg/Odometry` | 11 Hz | 車輪の回転から計算した位置・速度 |
| `/imu` | `sensor_msgs/msg/Imu` | 25 Hz | 加速度・角速度（傾きや旋回の検出） |
| `/scan` | `sensor_msgs/msg/LaserScan` | 12 Hz | LiDAR による周囲 360° の距離（360 点） |
| `/battery` | `std_msgs/msg/UInt16` | 1 Hz | 電池電圧。値 ÷ 10 が電圧 [V]（例：82 → 8.2 V） |
| `/espRos/esp32camera` | `sensor_msgs/msg/CompressedImage` | 約 7 Hz（実測） | カメラ画像（JPEG）。カメラモジュールが publish |

### PC → ロボット（subscribe）
| トピック | 型 | 役割・値 |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 走行指令。`linear.x` が前後 [m/s]、`angular.z` が旋回 [rad/s] |
| `/beep` | `std_msgs/msg/UInt16` | ブザー。0 = 停止、1 = 鳴り続ける、10 以上 = その ms だけ鳴る（最大 10000） |
| `/servo_s1` | `std_msgs/msg/Int32` | サーボ S1 の角度 [-90, 90]° |
| `/servo_s2` | `std_msgs/msg/Int32` | サーボ S2 の角度 [-90, 20]° |

### 実機で確かめた注意点
- **`/cmd_vel` は最後の指令を保持する**。1回送るだけで走り続け、タイムアウトで止まらない。自作ノードでは走行中に周期的に送り、終了時には停止を複数回送る
- **1通だけでは届かないことがある**（agent〜ESP32 間は WiFi の UDP）。停止やサーボ角度など、何回送っても結果が同じ命令は複数回送る（`ros2 topic pub -t 3 ...`）
- サーボは**絶対角度**で指定する。S1 は + が右、S2 は + が上
- LiDAR は 0°（`ranges` の 180 番目）が正面、角度の負が右。`range_min` は 0.12 m だが、実際は 0.05 m 程度まで返す。LiDAR の中心から車体の端までは前 0.113 m・後ろ 0.135 m
- **カメラのうなずき角が −35° より上だと、カメラの頭が LiDAR の測定面に入り、正面が見えなくなる**（−50° 以下なら見える）

### 出典
- Yahboom 配布物 `Factory-Firmware/ROS node topic information.pdf`（`/espRos/esp32camera` 以外）
- `/espRos/esp32camera` は実機で `ros2 topic list -t` を実行して確認
- 公式教材：[YahboomTechnology/Mirco-Ros-Car_VM](https://github.com/YahboomTechnology/Mirco-Ros-Car_VM)

## トラブルシューティング
| 症状 | 原因 | 対処 |
|---|---|---|
| ロボットのトピックが出ない | ロボットが Agent に接続していない。ESP32 は起動時にしか接続しない | Agent の起動後にロボットのリセットボタンを押す。`logs --since` で新しい接続を確認する（古いログも残っているため） |
| リセットしても接続が来ない | PC の IP が変わり、ロボットが古い IP に送っている（DHCP のリース切れ） | ルーターで PC に DHCP 固定割当を設定する。`ip -4 addr` で確認 |
| `ros2` コマンドが `TimeoutError: [Errno 110]` で落ちる | WSL2 mirrored では、未使用の `127.0.0.1` ポートへの接続がタイムアウトになる。そのため ros2 daemon の自動起動が失敗する | [compose.yaml](compose.yaml) で起動時に `ros2 daemon start` するよう対処済み。手動なら `ros2 daemon start` |
| `ros2 topic hz` で間隔がマイナスや約22秒になる | WSL 内の `systemd-timesyncd` が誤った NTP 結果で時計を動かし、Hyper-V の時刻同期がそれを戻す。この奪い合いで時計が数秒ごとに飛ぶ（`dmesg` に `Time jumped backwards`） | WSL で `sudo systemctl disable --now systemd-timesyncd`（WSL の時計は Hyper-V が同期する） |
| `/imu` などは届くのに `/scan`・カメラ画像だけ届かない（`ros2 topic echo` で `A message was lost`） | micro-ROS Agent の公式イメージは既定で共有メモリを無効にし（`MICROROS_DISABLE_SHM=1`）、ros コンテナへ UDP(127.0.0.1) で送る。WSL2 mirrored のループバックは 1472 バイトを超える UDP を捨てるので、大きいメッセージだけ消える | [compose.yaml](compose.yaml) で Agent に `MICROROS_DISABLE_SHM=0` を設定済み |
| 電源を入れてもロボットが接続しない | 不明（電源 ON だけでは接続しないことがある） | 起動して 30 秒ほど待ってからリセットボタンを押す |
| トピックが出るまで数秒かかる | DDS の discovery に時間がかかる | 数秒待つ。`ros2 topic list --no-daemon --spin-time 10` も可 |
| ブザーが鳴り続ける | 電池残量の低下 | 充電する（充電中は電源 OFF） |

- リセットボタンは電源スイッチと LiDAR コネクタの間にある。Serial 端子横の **BOOT ボタンは押さない**

## ライセンス
[MIT License](LICENSE)
