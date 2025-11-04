#!/usr/bin/env python3
# pid_right_hip.py : right_hip_joint を P制御（比例のみ）で effort 出力する最小ノード
import os, signal, subprocess
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray

class SingleJointP(Node):
    def __init__(self):
        super().__init__('single_joint_p')  # ノード名を宣言してROS2グラフに参加する。
        self.joint_name = 'right_hip_joint'  # 対象とする単関節の名前を指定する。

        self.Kp = 70.0           # Pゲイン（控えめに開始して後で上げ下げ調整する）。
        self.Ki = 25.0           # Iゲイン
        self.Kd = 5.0           # Dゲイン
        self.umax = 20.0         # 出力トルクの飽和を設定して危険な値を防ぐ。
        
        self.target = 0.0        # 目標角
        self.meas_pos = None     # 実角度
        self.meas_vel = 0.0      # 実角速度（今回は未使用）
        self.integ    = 0.0               # 積分状態
        self.imin, self.imax = -3.0, 3.0  # クランプ
        self.prev_t   = None              # dt計算用

        # 入出力
        self.sub_js  = self.create_subscription(JointState, '/biped/joint_states', self.on_js, 10)
        self.sub_ref = self.create_subscription(Float64, '/pid/target_position', self.on_ref, 10)
        self.pub_eff = self.create_publisher(Float64MultiArray, '/biped/effort_controller/commands', 10)

        # デバッグ可視化
        self.pub_tgt = self.create_publisher(Float64, '/pid/right_hip_joint/target', 10)
        self.pub_mp  = self.create_publisher(Float64, '/pid/right_hip_joint/meas_pos', 10)
        self.pub_mv  = self.create_publisher(Float64, '/pid/right_hip_joint/meas_vel', 10)
        self.pub_err = self.create_publisher(Float64, '/pid/right_hip_joint/error', 10)
        self.pub_u   = self.create_publisher(Float64, '/pid/right_hip_joint/effort', 10)
        self.pub_i   = self.create_publisher(Float64, '/pid/right_hip_joint/integ', 10)  # 積分状態を可視化

        # rqt_plot を自動起動（★ 別メソッドとして定義し、ここで呼ぶ）
        self.plot_proc = None
        self.spawn_rqt_plot()

        # 250 HzでP制御を実行（ros2_control の update_rate に合わせる）。
        self.timer = self.create_timer(1.0/250.0, self.on_timer)

    def spawn_rqt_plot(self):
        topics = [
            '/pid/right_hip_joint/target',
            '/pid/right_hip_joint/meas_pos',
            '/pid/right_hip_joint/meas_vel', 
            '/pid/right_hip_joint/error',
            '/pid/right_hip_joint/effort',
        ]
        cmd = ['ros2', 'run', 'rqt_plot', 'rqt_plot'] + topics
        try:
            self.plot_proc = subprocess.Popen(cmd)
            self.get_logger().info(f'Launched rqt_plot with topics: {", ".join(topics)}')
        except Exception as e:
            self.get_logger().warning(f'Failed to launch rqt_plot: {e}')

    def on_ref(self, msg: Float64):
        self.target = float(msg.data)  # 目標角の更新

    def on_js(self, msg: JointState):  # JointStateから対象関節の角度/速度を取り出す。
        try:
            idx = msg.name.index(self.joint_name)
            self.meas_pos = msg.position[idx]
            if len(msg.velocity) > idx:
                self.meas_vel = msg.velocity[idx]
        except ValueError:
            pass  # 対象関節が含まれないメッセージは無視する。

    def on_timer(self):  # 250Hzタスク：P制御
        if self.meas_pos is None:
            return
            
        # dt計算
        now = self.get_clock().now().nanoseconds * 1e-9
        dt = 0.0 if self.prev_t is None else (now - self.prev_t)
        self.prev_t = now
        if dt <= 0.0:
            dt = 0.0
             
        error = self.target - self.meas_pos         # 誤差 = 目標 - 現在
        cand_integ = self.integ + error * dt        # 候補の積分（まず更新してみる）
        u_unsat = self.Kp*error - self.Kd*self.meas_vel + self.Ki*cand_integ    # 未飽和出力で評価
        u = min(max(u_unsat, -self.umax), self.umax)# 出力飽和

        # ★ anti-windup：飽和方向に押し続けるときは積分を進めない
        if not ((u != u_unsat) and ((u > 0 and error > 0) or (u < 0 and error < 0))):
            # クランプして採用
            self.integ = min(max(cand_integ, self.imin), self.imax)
        
        self.pub_eff.publish(Float64MultiArray(data=[u]))  # 単関節なので [u]

        # 可視化出力
        self.pub_tgt.publish(Float64(data=self.target))
        self.pub_mp .publish(Float64(data=self.meas_pos))
        self.pub_mv .publish(Float64(data=self.meas_vel))
        self.pub_err.publish(Float64(data=error))
        self.pub_u  .publish(Float64(data=u))

    def destroy_node(self):  # 終了時に rqt_plot も丁寧に終了
        try:
            if self.plot_proc and self.plot_proc.poll() is None:
                self.get_logger().info('Terminating rqt_plot...')
                self.plot_proc.send_signal(signal.SIGINT)
        except Exception:
            pass
        finally:
            super().destroy_node()

def main():
    rclpy.init()
    node = SingleJointP()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

