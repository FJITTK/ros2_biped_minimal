# Minimal Biped (ROS 2 Humble + Gazebo Classic)

## 1. 環境・前提
~~~
OS: Ubuntu (WSL2可)
ROS: ROS 2 Humble
Simulator: Gazebo Classic（Ignition/Garden記法は不使用）
主要パッケージ: gazebo_ros, robot_state_publisher, ros2_control
本リポ構成（参考・固定）：
~/ros2_ws/
├─ src/
│ ├─ my_robot_description/
│ │ ├─ config/
│ │ │ ├─ ros2_control.yaml # コントローラ設定（gazebo_ros2_control が読む）
│ │ │ └─ rsp_params.yaml # /biped/robot_state_publisher のパラメータ
│ │ └─ urdf/
│ │ ├─ minimal_biped.urdf # 使用中URDF
│ │ ├─ minimal_biped.noctrl.urdf
│ │ ├─ minimal_biped.urdf.bak.20251005_0019
│ │ ├─ minimal_robot.urdf
│ │ └─ minimal_robot.sdf
│ └─ my_robot_bringup/
│ └─ launch/
│ └─ biped_bringup.launch.py # Gazebo/RSP/spawn/spawner の統合起動
├─ build/ ...
├─ install/ ...
└─ log/ ...
~~~

## 2. クイックスタート
~~~
cd ~/ros2_ws
colcon build --packages-select my_robot_description my_robot_bringup --symlink-install
source install/setup.bash
ros2 launch my_robot_bringup biped_bringup.launch.py

launchは Gazebo → RSP → spawn → spawner（JSB → position系コントローラ）の順で起動する設計。
~~~

## 3. マイルストン一覧（S0→S5）

| Step | 何をする | 知識 | クリア条件 | ステータス | 成果物（例） |
|---|---|---|---|---|---|
| **S0** 静置健全性 | URDFの質量・慣性・ダンピング見直し、置いただけで暴れない状態に | 剛体動力学、数値安定 | 10秒以上、姿勢が崩れない | ☐ | URDF差分、動画、log |
| **S1** 単関節テスト | `ros2_control.yaml`の`joints`を1–2関節に限定（例：膝±5°往復） | 離散PD、制御周期（`update_rate`） | 振動せず停止、RMS誤差小 | ✅ | bag、rqt_plot、PID値 |
| **S2a** 2D化（空中） | `world_to_torso`固定で左右6関節（pitch×6）PID追従 | LIPM入門、PID整定 | rqt/bagで再現OK | ✅ | 6関節追従ログ、動画 |
| **S2b** 2D化（接地） | 胴体固定解除、footに接触surface、`kp/kd`調整で滑り/刺さり/過反発を抑制 | 接触モデル、摩擦 | 滑り・刺さり・過反発を抑制 | ✅ | 足裏力・速度ログ、動画 |
| **S3** 2.5D: 片脚スイング | 支持脚を硬く、遊脚の5次多項式軌道を生成・追従 | 軌道生成、衝撃緩和 | 数歩の“足踏み歩行”（前進なし）が安定 | ☐ | 軌道CSV、再現動画 |
| **S4** y解放: 3D LIPM/CP | 横方向のバランス（足先配置）制御を導入 | Capture Point、支持多角形、ZMP | 0.1–0.2 m/sで5–10歩継続 | ☐ | 足先配置ログ |
| **S5** Whole-Body化 | QP/Stack-of-TasksでCoM・足・骨盤・姿勢のタスク優先制御 | 逆動力学、二次計画、接触制約 | 速度変更・小外乱に耐え、安定停止 | ☐ | QP設定、比較動画 |

**凡例**：✅ 完了 / ☐ 未了


## 6. 成果物・ログの置き場
~~~
repo_root/
├─ docs/
│ ├─ experiments/
│ │ ├─ S1_2025-11-04_run01_kpXX_kdYY.md
│ │ └─ S2b_2025-11-05_run03_mu06_kp10_kd03.md
│ └─ tuning.md # しきい値変更・PID・摩擦の履歴
├─ data/
│ ├─ bags/ # rosbag
│ └─ plots/ # 画像（rqt_plotのエクスポート等）
└─ scripts/ # 実験補助スクリプト（将来）
~~~

## 7. コントローラ/URDFの参照
~~~
URDF: ~/ros2_ws/src/my_robot_description/urdf/minimal_biped.urdf
RSP: ~/ros2_ws/src/my_robot_description/config/rsp_params.yaml
ros2_control: ~/ros2_ws/src/my_robot_description/config/ros2_control.yaml
Launch: ~/ros2_ws/src/my_robot_bringup/launch/biped_bringup.launch.py
~~~
