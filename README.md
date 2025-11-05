# Minimal Biped (ROS 2 Humble + Gazebo Classic)

## 環境（版）
OS: Ubuntu (WSL2可)
ROS: ROS 2 Humble
Simulator: Gazebo Classic（Ignition/Garden記法は不使用）
主要パッケージ: gazebo_ros, robot_state_publisher, ros2_control
本リポ構成（参考・固定）：

## 配置ツリー（要点のみ）

~~~
src/my_robot_description/{urdf,config}/...
src/my_robot_bringup/launch/biped_bringup.launch.py
~~~

## 使い方（3行）
~~~bash
colcon build --packages-select my_robot_description my_robot_bringup --symlink-install
source install/setup.bash
ros2 launch my_robot_bringup biped_bringup.launch.py
~~~

## 依存関係の取得（必要なら）
~~~bash
rosdep update
rosdep install -r --from-paths src -i -y
~~~
