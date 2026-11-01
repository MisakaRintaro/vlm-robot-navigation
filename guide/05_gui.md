# 05. GUI で観る

講座（応用編1 第4・5回）で使った可視化ツールで、ロボットのデータを見る。

## ゴール
- `rqt_graph` でノードとトピックのつながりを見る
- `rqt_image_view` でカメラ映像を見る
- `rqt_plot` でセンサの値をグラフで見る
- `rviz2` で LiDAR の点群を見る

## 準備（済）
- [../docker/Dockerfile](../docker/Dockerfile) に `rqt-graph`・`rqt-image-view`・`rqt-plot`・`rviz2`・`image-transport-plugins` を追加
- [../compose.yaml](../compose.yaml) で WSLg の画面に表示できるようにした（`DISPLAY` などの環境変数と `/mnt/wslg` のマウント）

`QStandardPaths: runtime directory ... is not owned by UID 0` という警告は表示されるが、無視してよい。

## ① rqt_graph：ノードとトピックのつながり
```bash
# ターミナル1
cd /ws/src/basics && python3 battery_monitor.py
# ターミナル2
ros2 run rqt_graph rqt_graph
```
- `/YB_Car_Node` → `/battery` → `/battery_monitor` のつながりが図で見える
- 左上を「Nodes/Topics (all)」にすると、すべてのトピックが表示される
- 図が古いときは左上の更新ボタンを押す

## ② rqt_image_view：カメラ映像
カメラは JPEG（`CompressedImage`）を `/espRos/esp32camera` に送っている。`rqt_image_view` で選べるように、通常の画像（`Image`）に変換してから表示する。
```bash
# ターミナル1：JPEG → 通常の画像に変換して /camera/image_raw に流す
ros2 run image_transport republish compressed raw --ros-args \
  -r in/compressed:=/espRos/esp32camera -r out:=/camera/image_raw
# ターミナル2
ros2 run rqt_image_view rqt_image_view
```
- 左上のプルダウンで `/camera/image_raw` を選ぶ
- 解像度は 320×240、約 7 Hz
- ステップ2のサーボ（`/servo_s1`, `/servo_s2`）を動かすと、映像の向きが変わる

## ③ rqt_plot：センサ値の時系列グラフ
```bash
# IMU の加速度（ロボットを傾けると線が入れ替わる）
ros2 run rqt_plot rqt_plot /imu/linear_acceleration/x /imu/linear_acceleration/y /imu/linear_acceleration/z
# 走行中の速度（ステップ4の drive_forward.py と一緒に。加速→一定→停止の形が見える）
ros2 run rqt_plot rqt_plot /odom_raw/twist/twist/linear/x
```
- 引数は「トピック名 + フィールドのパス」を `/` でつなげたもの（`--field linear_acceleration.x` と同じ場所を指す）
- 画面上部の入力欄からも追加できる

## ④ rviz2：LiDAR の点群
```bash
rviz2
```
1. 左の「Global Options」→「Fixed Frame」に `laser_frame` と入力する（`/scan` の `header.frame_id`）
2. 左下の「Add」→「By topic」→ `/scan` の「LaserScan」を選ぶ
3. 周囲の壁や物が点で表示される。手をかざすと点が動く
4. 点が小さければ、LaserScan の「Size (m)」を 0.03 くらいにする

- ステップ1で見た `header.frame_id` は、ここで「どの座標系に描くか」を決めるのに使われる

## rqt のほかのプラグイン（必要になったら Dockerfile に追加する）
| プラグイン | 何をするか |
|---|---|
| rqt_robot_steering | スライダーで `/cmd_vel` を送る |
| rqt_topic | トピック一覧・中身・周期を表で表示（`ros2 topic` の GUI 版） |
| rqt_publisher | GUI でメッセージを送る（`ros2 topic pub` の GUI 版） |
| rqt_console | ノードのログを一覧・絞り込み |
| rqt_tf_tree | 座標系（TF）のつながりを図で表示 |
| rqt_bag | rosbag の中身を時系列で見る |

`rqt` だけを起動して「Plugins」メニューから選び、1つの画面に並べることもできる。

## 結果メモ
- ① rqt_graph：初期設定では受け取り手のいないトピックが隠れる。「Nodes/Topics (all)」にして「Hide:」のチェックを外すと全体が見える。`/rqt_gui_py_node_*`・`/_ros2cli_daemon_*`・`/rosout`・`/parameter_events` はシステム関係（「Hide: Debug」で隠せる）
- ② rqt_image_view：起動しただけでは何も映らない。左上のプルダウンで `/camera/image_raw` を選ぶ（なければ更新ボタン）と映った
- ③ rqt_plot：起動直後は表示範囲が合わず何も見えない。表示範囲を広げる（Home・Zoom・Figure options）と見えた。1秒の走行は一瞬なので、走行後に autoscroll を外して拡大する
- ④ rviz2：Fixed Frame は「Global Options」を展開し、値の `map` をクリックして `laser_frame` に変える。`map` はこのロボットに存在しないのでエラーになる
  - Fixed Frame ＝ 画面の原点にする座標系。`/scan` の `frame_id` と同じにすれば変換なしで描ける
  - このロボットは `/tf`（座標系どうしの関係）を出していないため、LiDAR 単体の表示になる。地図作り（SLAM）などには TF が必要
