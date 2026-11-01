# 03. Subscriber：電池電圧を表示する

初めての Python ノード。`ros2 topic echo /battery` と同じことを自分で書く。
書き方は講義（応用編1 第4回）の Subscriber と同じ。

## ゴール
- Subscriber ノードの基本形（`Node` を継承 → `create_subscription` → コールバック → `spin`）を理解する

## ファイル
| ファイル | 内容 |
|---|---|
| [../src/basics/battery_monitor.py](../src/basics/battery_monitor.py) | ノード本体 |
| [../src/basics/test_battery_monitor.py](../src/basics/test_battery_monitor.py) | 電圧変換のテスト |

## 実行
```bash
cd /ws/src/basics            # ros コンテナの中
python3 battery_monitor.py   # 1秒ごとに "8.1 V" のように表示される（Ctrl+C で終了）
python3 -m pytest -q         # テスト
```

## コードの流れ
```
main()
 ├─ rclpy.init()                 ROS 2 を使う準備
 ├─ BatteryMonitor()             ノードを作る
 │    └─ create_subscription(UInt16, '/battery', on_battery, 10)
 │                               「/battery が届いたら on_battery を呼んで」と登録
 ├─ rclpy.spin(node)             届くのを待ち続ける。届くたびに on_battery(msg) が呼ばれる
 └─ destroy_node / try_shutdown  後片付け
```

| 部分 | ポイント |
|---|---|
| `create_subscription` の引数 | 型・トピック名・コールバック・キューサイズ。**型とトピック名はステップ1で調べたもの** |
| コールバック `on_battery(msg)` | `msg.data` は `ros2 topic echo --field data` と同じパス |
| `spin` | 自分では while ループを書かない。ROS が届くたびに呼び出してくれる |
| キューサイズ 10 | 処理が追いつかないときに溜めておける数。最新値だけでよければ 1 |
| `to_volts` を分けた理由 | ROS なしでテストできるようにするため（変換ロジックと通信を分ける） |

- QoS：ロボット側の `/battery` は `RELIABLE`（`ros2 topic info /battery --verbose` で確認）。標準設定のままで受け取れる

## 試してみよう
- `ros2 topic echo /battery` と並べて実行し、同じタイミングで届くことを確かめる
- 実行中に別のターミナルで `ros2 node list` を実行する → 自分のノード `/battery_monitor` が見える
- 実行中に `ros2 topic info /battery` を実行する → Subscription count が 1 増えている

## 用語
| 用語 | 意味 |
|---|---|
| ノード | 1つの役割を持った処理のひと塊。publish / subscribe の主体で名前を持つ（`ros2 node list`）。Python では `Node` を継承したクラス1つ ≒ ノード1つ |
| `rclpy.init()` | ROS 2 通信の準備（DDS の初期化、`ROS_DOMAIN_ID` などの読み込み、Ctrl+C 処理の登録）。最後の `shutdown` とペア |
| キュー | 届いたメッセージを処理するまで溜める待ち行列。あふれると古いものから捨てる。最新値だけ欲しいなら 1、標準は 10、取りこぼしたくないなら 100 など |

## 結果メモ
- 1秒ごとに 8.1 V と表示された（公式資料の 1 Hz と一致）
- 試してみよう の3つを確認した（echo と同じタイミングで届く／`/battery_monitor` が node list に出る／Subscription count が増える）
