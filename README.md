# wall_tracking [![build-test](https://github.com/makotoyoshigoe/wall_tracking/actions/workflows/build-test.yaml/badge.svg)](https://github.com/makotoyoshigoe/wall_tracking/actions/workflows/build-test.yaml)
左側にある壁や物体と一定の距離を保ちながらロボットを走行させるためのパッケージです。

# デモ動画
https://user-images.githubusercontent.com/91446273/225591825-b1945d4b-a7fb-4d5a-9edf-96e4bbe11918.mp4

# Nodes
## wall_tracking_node
このノードはLiDARからのスキャンデータを受け取り、そのうちの69[deg]から78[deg]のデータを使用し、PID制御によってそれらの平均値が目標値になるようにロボットに速度指令を送るノードです。
### Subscribed Topics
- scan([sensor_msgs/msg/LaserScan](https://docs.ros2.org/foxy/api/sensor_msgs/msg/LaserScan.html))
    - LiDARのスキャンデータ
### Published Topics
- {robot_name}/cmd_vel([geometry_msgs/msg/Twist](https://docs.ros2.org/latest/api/geometry_msgs/msg/Twist.html))
    - ロボットの速度指令値
    - robot_nameはパラメータ(~robot_name)で設定
### [Parameters](https://github.com/makotoyoshigoe/wall_tracking/blob/master/config/wall_tracking.param.yaml)
- ~robot_name (string, default: "turtlebot3")
    - ロボットの名前
- ~distance_from_wall (double, default: 0.4[m])
    - 左側の壁や物体との距離の目標値
- ~distance_to_stop (double, default: 0.5[m])
    - 前方の物体との距離がこの値以下になると停止する
- ~max_linear_vel(double, default: 0.22[m/s])
    - 並進速度の最大値
- ~max_angular_vel (double, default: 2.84[rad/s])
    - 角速度の最大値
- ~min_angular_vel (double, default: -2.84[rad/s])
    - 角速度の最小値
- ~sampling_rate (double, default: 0.033[s])
    - LiDARのサンプリング周期
- ~kp (double, default: 4.0)
    - Pゲイン
- ~ki (double, default: 0.01)
    - Iゲイン
- ~kd (double, default: 0.25)
    - Dゲイン
## 動作環境
- ROS2 Humble (Ubuntu 22.04.1 LTS)

## デモ動画（応用）

https://user-images.githubusercontent.com/91446273/225596368-cbe41cf1-149f-4e4d-b7e6-2a2fa27e1b88.mp4
