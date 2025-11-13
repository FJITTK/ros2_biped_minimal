# swing_demo.py（最小骨組み：起動確認用）

# 1) rclpyを使うための標準import
import rclpy                     # ← ROS 2のPythonクライアント
from rclpy.node import Node      # ← Nodeクラス（ノードの基本形）

def main():
    rclpy.init()                 # ← ROS通信の初期化
    node = Node('swing_demo')    # ← ノード作成（名前: swing_demo）

    node.get_logger().info('swing_demo: 起動テストOK')  # ← 1行だけログ

    node.destroy_node()          # ← ノードを片付け
    rclpy.shutdown()             # ← ROS通信の終了

