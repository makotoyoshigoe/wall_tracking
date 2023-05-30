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
- ~distance_from_wall (double, default: 0.7[m])
    - 左側の壁や物体との距離の目標値
- ~distance_to_stop (double, default: 0.7[m])
    - 前方の物体との距離がこの値以下になると停止する
- ~max_linear_vel (double, default: 0.25[m/s])
    - 並進速度の最大値
- ~max_angular_vel (double, default: 0.7[rad/s])
    - 角速度の最大値
- ~min_angular_vel (double, default: -0.7[rad/s])
    - 角速度の最小値
- ~sampling_rate (double, default: 0.033[s])
    - LiDARのサンプリング周期
- ~kp (double, default: 12.0)
    - Pゲイン
- ~ki (double, default: 0.0)
    - Iゲイン
- ~kd (double, default: 0.0)
    - Dゲイン
- ~start_deg_lateral (int, default: 69[deg])
    - サンプリングするレーザーの開始角度
- ~end_deg_lateral (int, default: 78[deg])
    - サンプリングするレーザーの終了角度
- ~ray_th (int, default: 16)
    - {distance_to_stop}[m]以下になった前方のレーザーが{ray_th}本以上になると90[deg]旋回する
- ~wheel_separation (double, default: 0.28[m])
    - タイヤ間の距離
- ~distance_to_skip (double, default: 1.0[m])
    - 隙間とみなす距離
## 動作環境
- ROS2 Humble (Ubuntu 22.04.1 LTS)

## デモ動画（応用）

https://user-images.githubusercontent.com/91446273/225596368-cbe41cf1-149f-4e4d-b7e6-2a2fa27e1b88.mp4
