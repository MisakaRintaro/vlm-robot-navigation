# 04. Publisher：1秒前進して止まる

`/cmd_vel` を publish してロボットを動かすノード。ステップ2で `ros2 topic pub` を使ってやったことを、コードで書く。

## ゴール
- Publisher ノードの基本形（`create_publisher` → タイマーで周期的に `publish`）を理解する
- ステップ2でわかった「`/cmd_vel` は最後の指令を保持する」「1通では届かないことがある」に対して、安全な作りにする

## ファイル
| ファイル | 内容 |
|---|---|
| [../src/basics/drive_forward.py](../src/basics/drive_forward.py) | ノード本体 |
| [../src/basics/test_drive_forward.py](../src/basics/test_drive_forward.py) | 経過時間と速度の関係のテスト |

## 実行（車輪を浮かせて）
```bash
cd /ws/src/basics
python3 drive_forward.py    # 0.1 m/s で1秒前進して止まり、自動で終了する
```
- 別のターミナルで `ros2 topic echo /cmd_vel --field linear.x` を開いておくと、送った値が見える
- ロボットを動かさずに試すときは、トピック名を付け替える：`python3 drive_forward.py --ros-args -r /cmd_vel:=/cmd_vel_test`

## `create_publisher` の意味
```python
self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
#    ①                   ②             ③       ④          ⑤
```
| # | 部分 | 意味 |
|---|---|---|
| ① | `self.publisher =` | 作った Publisher を保存しておく。あとで `self.publisher.publish(msg)` で送るため |
| ② | `self.create_publisher(...)` | このノードに「送信口」を作る（ノードの機能なので `self.`） |
| ③ | `Twist` | 送るメッセージの型（`geometry_msgs/msg/Twist`）。受け取る側（ロボット）と同じ型にする |
| ④ | `'/cmd_vel'` | 送り先のトピック名 |
| ⑤ | `10` | キューサイズ。送り切れていないメッセージを最大10通まで保持する（再送や、受け取る側の準備待ちに使う） |

ステップ2のコマンドと対応している。
```
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"
               ④         ③                     ← 中身は on_timer で msg.linear.x = 0.1 として作る
```

Subscriber（ステップ3）との違い：
| | Subscriber | Publisher |
|---|---|---|
| 作り方 | `create_subscription(型, トピック, コールバック, キュー)` | `create_publisher(型, トピック, キュー)` |
| 送受信 | 届くと**コールバックが呼ばれる** | **自分で `publish(msg)` を呼ぶ** |
| 保存 | 不要なことが多い | `publish` するために変数に保存する |

## コードの流れ
```
main()
 ├─ rclpy.init(signal_handler_options=NO)   ROS の Ctrl+C 処理を無効にする（理由は後述）
 ├─ DriveForward()
 │    ├─ create_publisher(Twist, '/cmd_vel', 10)
 │    └─ create_timer(0.1 秒, on_timer)    0.1秒ごとに on_timer を呼ぶよう登録
 ├─ while not done: spin_once(0.1秒)       タイマーを動かす。1秒経つと done になる
 └─ finally: stop()                         正常終了でも Ctrl+C でも、停止を確認できるまで送る
```

| 部分 | ポイント |
|---|---|
| `create_publisher(型, トピック名, キュー)` | Subscriber と同じく、型とトピック名はステップ1・2で調べたもの |
| `create_timer(周期, 関数)` | 一定周期で関数を呼ぶ。自分で `sleep` のループを書かない |
| `on_timer` で毎回 publish | ロボットは最後の指令を保持し、1通落ちることもあるので、走行中も 10 Hz で送り続ける |
| `velocity_at` を分けた理由 | 「いつ・何 m/s で走るか」を ROS なしでテストするため |
| `finally: node.stop()` | プログラムがどう終わっても必ず止める |
| `stop()` の停止確認 | `/odom_raw`（車輪から計算した実際の速度）がほぼ 0 になるまで、0.1秒ごとに停止を送る。2秒で確認できなければ警告を出して終わる |
| `is_stopped` を分けた理由 | 「止まったとみなす基準（0.01 未満）」を ROS なしでテストするため |

## Ctrl+C で停止を送るための工夫
- Humble 標準の Ctrl+C 処理は、**先に通信（コンテキスト）を閉じる**。そのあとで publish すると `publisher's context is invalid` エラーになり、停止指令を送れない
- `on_shutdown` のコールバックも、通信を閉じた後に呼ばれるので使えない
- そこで `SignalHandlerOptions.NO` で ROS の Ctrl+C 処理を無効にし、Python の `KeyboardInterrupt` で受ける
- ただし `rclpy.spin()` は Python に制御が戻らないので、Ctrl+C で抜けられなくなる。そのため 0.1 秒の `spin_once` を繰り返す

## 確認済み（トピックを付け替えて実施）
| ケース | 送られたメッセージ |
|---|---|
| 正常終了 | `0.1` が約10回 → `0.0` を停止確認まで送り「停止を確認しました」 |
| 走行中に Ctrl+C | `0.1` が続いたあと → `0.0` を停止確認まで送る |
| odom が届かない | 2秒後に「停止を確認できませんでした」と警告して終了 |

## 試してみよう
- `SPEED` や `DURATION` を変えてみる（速くしすぎない。0.2 m/s 程度まで）
- 床に置いて走らせ、本当に約 10 cm（0.1 m/s × 1 s）進むか測る
- 実行中に `ros2 topic info /cmd_vel --verbose` を実行し、Publisher に `drive_forward` が出ることを確かめる

## 停止の確認（オープンループ → フィードバック）
最初は「停止を3回送る」だけだった（送りっぱなし＝オープンループ）。本当に止まったかを確かめるため、`/odom_raw` を subscribe して、速度が 0 になるまで送るようにした（結果を見て判断する＝フィードバック）。
- 1つのノードで Publisher（`/cmd_vel`）と Subscriber（`/odom_raw`）を組み合わせている
- `/odom_raw` のキューは 1（古い速度で判断しないため）
- 停止指令を送る前の速度で判定しないように、`stop()` の最初に古い値を捨てる

`/odom_raw` の `twist.twist.linear.x` を走行中に見た結果：
| 区間 | 値 | 読み取れること |
|---|---|---|
| 走り始め | 0.004 → 0.065 → 0.087 | 約0.4秒かけて加速する |
| 走行中 | 0.098〜0.112 | 指令どおり約 0.1 m/s |
| 停止後 | 0.018 → 0.0 | 止まった |

## 結果メモ
- 車輪を浮かせて実行し、1秒回って止まった
- 停止確認つきで実行し、「停止を確認しました」と表示された
