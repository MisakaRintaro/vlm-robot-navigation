# ロボット操作用の ROS 2 Humble 環境
# Yahboom 公式VM と同じ Humble に揃え、標準トピック(/cmd_vel, /scan 等)だけで操作する
FROM ros:humble-ros-base

# キーボード操縦と、可視化ツール（rqt_graph / rqt_image_view / rqt_plot / rviz2）
# image-transport-plugins は rqt_image_view で JPEG（CompressedImage）を表示するために必要
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ros-humble-teleop-twist-keyboard \
        ros-humble-rqt-graph \
        ros-humble-rqt-image-view \
        ros-humble-rqt-plot \
        ros-humble-rviz2 \
        ros-humble-image-transport-plugins \
    && rm -rf /var/lib/apt/lists/*

# Gemini API（VLM）を Python から呼ぶための公式 SDK。
# pytest は 7 系にする（SDK と一緒に入る anyio のプラグインは 7 以上が必要。
# 一方 ROS 2 Humble の launch_testing プラグインは 8 以降に非対応）
RUN apt-get update \
    && apt-get install -y --no-install-recommends python3-pip \
    && rm -rf /var/lib/apt/lists/* \
    && pip3 install --no-cache-dir google-genai "pytest>=7,<8"

# 対話シェルで ROS 2 コマンドをすぐ使えるようにする
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

WORKDIR /ws
