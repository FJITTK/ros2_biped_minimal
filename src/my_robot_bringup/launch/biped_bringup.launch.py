from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_bringup = get_package_share_directory('my_robot_bringup')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_description = get_package_share_directory('my_robot_description')
    
    # Gazebo Classic をプラグイン明示 + Pause で起動
    empty_world = os.path.join(pkg_gazebo_ros, 'worlds', 'empty.world')
    gazebo_launch = ExecuteProcess(
        cmd=[
            'gzserver', empty_world,
            '--verbose', '--pause',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so'
        ],
        output='screen'
    )

    urdf_file = os.path.join(pkg_description, 'urdf', 'minimal_biped.urdf')
    rsp_params = os.path.join(pkg_description, 'config', 'rsp_params.yaml')

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='biped',
        parameters=[
            rsp_params,
            {'robot_description': open(urdf_file).read()}
        ],
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'biped',
            '-file', urdf_file,
            '-x','0','-y','0','-z','0.37'
            ,'-timeout','120'
        ],
        output='screen'
    )
    
    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )

    # JointStateBroadcaster は先に起動（/biped/joint_states を出す）
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/biped/controller_manager', '--inactive', '--controller-manager-timeout', '20'],  # この名前はYAMLと一致
        output='screen'
    )

    # ★PID入りのJTC（effort駆動）を起動：YAMLで定義した 'leg_jtc' をspawn
    leg_jtc_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['leg_jtc', '-c', '/biped/controller_manager', '--inactive', '--controller-manager-timeout', '20'],  # managerは /biped 配下
        output='screen'
    )
    
    # spawn と controller spawner を 3 秒遅延して、/spawn_entity の準備完了を待つ
    delayed_actions = TimerAction(
        period=5.0,
        actions=[
            spawn_entity,
            joint_state_broadcaster_spawner,
            leg_jtc_spawner
        ],
    )


    return LaunchDescription([
        gazebo_launch,
        gzclient,
        rsp_node,
        delayed_actions
    ])

