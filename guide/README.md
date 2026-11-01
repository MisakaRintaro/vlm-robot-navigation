# 実機の基礎 学習ガイド

課題に入る前に、ロボットを ROS 2 で「観る・動かす」基礎を、簡単なものから順に学んだ記録。
構成・起動手順・トピック一覧・トラブル対処は [../README.md](../README.md) を参照。

## 進め方
| # | 内容 | ドキュメント | 状態 |
|---|---|---|---|
| 1 | CLI で観る（`ros2 topic type / echo / hz`） | [01_cli_observe.md](01_cli_observe.md) | 済 |
| 2 | CLI で動かす（`ros2 topic pub` でブザー・サーボ・走行） | [02_cli_move.md](02_cli_move.md) | 済 |
| 3 | Subscriber：電池電圧を表示するノード | [03_subscriber.md](03_subscriber.md) | 済 |
| 4 | Publisher：1秒前進して停止するノード | [04_publisher.md](04_publisher.md) | 済 |
| 5 | GUI で観る（rqt_graph / rqt_image_view / rviz2） | [05_gui.md](05_gui.md) | 済 |
| 6 | Sub + Pub：LiDAR で前方の障害物を見て止まる | 未作成 | |
| 7 | 記録と再生（`ros2 bag record / play`） | 未作成 | |

## 講座資料との対応（応用編1）
| ツール | わかること | 講義 |
|---|---|---|
| `ros2 topic list / echo / type / hz` | トピック一覧・中身・型・周期 | 第4・5回 |
| `rqt_graph` | ノードとトピックのつながり | 第4・5回 |
| `rqt_image_view` | カメラ映像 | 第5回 |
| `rviz2` | LiDAR・オドメトリの3D表示 | 第4回 |
| `ros2 bag` / rerun | 記録・再生と時系列の可視化 | 第5回 |

## 基礎知識

### コンテナとシェル
- **コンテナ**：独立した小さな Linux 環境。3つとも裏で動き続けている
- **シェル**：コンテナに入ってコマンドを打つ画面。`docker compose exec ros bash` で開く
- 作業で入るのは `ros` コンテナだけ。`agent` と `camera-agent` は起動したら放っておく
- シェルはターミナルを複数開けば同時にいくつでも入れる。`exit` で出てもコンテナは動き続ける

| プロンプト | 今いる場所 |
|---|---|
| `user@host:~$` | WSL（ROS 2 なし。`ros2` は `command not found`） |
| `root@...:/ws#` | `ros` コンテナ（ROS 2 あり） |

### ros コンテナと agent の関係
`ros` コンテナは agent と直接やりとりしない。agent はロボットのデータを**トピックとして publish** する。`ros` 側はそのトピックを **subscribe** するだけでよい（第4回の Publisher / Subscriber と同じ）。

### トピック名と型
| | 例 | たとえ |
|---|---|---|
| トピック名 | `/battery` | テレビのチャンネル |
| 型 | `std_msgs/msg/UInt16` | そのチャンネルで流れる番組の形式 |

- 型の読み方：`std_msgs`（パッケージ）/ `msg`（トピック用）/ `UInt16`（型名）
- **1つのトピックの型は1つ**。Publisher と Subscriber は同じ型を使う
- 「入力 / 出力」はロボットから見た向き。トピック自体は常に Publisher → Subscriber の一方向
- データを送っているのはセンサそのものではなく ESP32 のノード `/YB_Car_Node`

| コマンド | わかること |
|---|---|
| `ros2 topic list -t` | トピック一覧と型 |
| `ros2 interface show std_msgs/msg/UInt16` | 型のフィールド（`uint16 data`） |
| `ros2 topic info /cmd_vel --verbose` | Publisher / Subscriber のノード |
| `ros2 node list` | ノード一覧 |

### 知らないトピックの調べ方（どのロボットでも使える）
```bash
ros2 topic list -t                              # ① どんなトピックがあるか、型は何か
ros2 interface show sensor_msgs/msg/Imu         # ② その型にどんなフィールドがあるか
ros2 topic echo /imu --field angular_velocity   # ③ 見たいフィールドだけを見る
ros2 topic echo -h                              # コマンドのオプションを調べる
```

| 知りたいこと | 情報源 | ロボットが違っても同じ？ |
|---|---|---|
| フィールド（`x, y, z` など） | 型の定義（`ros2 interface show`） | 同じ（標準型の場合） |
| コマンド・オプション | `-h` のヘルプ、ROS 2 公式ドキュメント | 同じ |
| 単位・軸の向き | [REP 103](https://www.ros.org/reps/rep-0103.html)（SI 単位、x=前・y=左・z=上） | 原則同じ |
| トピック名・周期・独自の決まり（電圧 ÷10 など） | メーカーの資料 | ロボットごとに違う |

`sensor_msgs/msg/Imu` のような標準型なら、メーカーが違ってもフィールドは同じ。自作コードもトピック名を変えるだけで流用できる。

### ROS 2 の実行方法
| 方法 | 用途 |
|---|---|
| `ros2 ...` コマンド | 観察・デバッグ用の道具（ターミナルから使う） |
| `python3 xxx.py` | 自作ノードをすぐ試す。準備不要 |
| `ros2 run <pkg> <node>` | パッケージ化したノードを名前で実行。`colcon build` が必要。配布・launch・再現性に向く |

どの方法も **`ros` コンテナの中**で実行する（`rclpy` はコンテナにしか入っていない）。学習中は `python3`、課題ではパッケージ化して `ros2 run` を使う。

### IMU
Inertial Measurement Unit（慣性計測装置）。スマホの画面回転と同じ種類のセンサ。
| フィールド | わかること |
|---|---|
| `linear_acceleration` | 静止時は重力の向き（＝傾き）。動けば加減速 |
| `angular_velocity` | どの軸周りに、どれだけ速く回転しているか |
