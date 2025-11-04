# Minimal Biped (ROS 2 Humble + Gazebo Classic)

## 環境（版）
- Ubuntu 22.04 / ROS 2 Humble / Gazebo Classic

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
