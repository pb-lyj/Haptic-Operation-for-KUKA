import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_lbr = get_package_share_directory('lbr_demos_advanced_py')
    admittance_param = os.path.join(pkg_lbr, 'config', 'admittance_control.yaml')

    return LaunchDescription([

        # 启动机械臂 bringup（非仿真）
        ExecuteProcess(
            cmd=[
                'ros2', 'launch', 'lbr_bringup', 'bringup.launch.py',
                'sim:=false',
                'ctrl:=lbr_joint_position_command_controller',
                'model:=iiwa14'
            ],
            shell=True
        ),

        # 启动 admittance_control 控制器，设置 namespace 为 /lbr
        Node(
            package='lbr_demos_advanced_py',
            executable='admittance_control',
            namespace='lbr',
            name='admittance_control',
            parameters=[admittance_param],
            output='screen'
        ),

        # 启动Launch控制的数据采集器节点
        Node(
            package='haptic',
            executable='dataset_recorder_launch',
            name='dataset_recorder',
            output='screen'
        ),

        # 启动 Tac3D 后端服务进程（C++运行库）
        ExecuteProcess(
            cmd=[
                './Tac3D', '-c', 'config/A1-0040R', '-d', '2', '-i', '127.0.0.1', '-p', '9988'
            ],
            cwd='/home/lyj/robot_space_2/ros2_driver_layer/src/Tac3D-v3.1.3-linux',
            shell=True,
            output='screen'
        ),
        
        ExecuteProcess(
            cmd=[
                './Tac3D', '-c', 'config/A1-0041L', '-d', '2', '-i', '127.0.0.1', '-p', '9989'
            ],
            cwd='/home/lyj/robot_space_2/ros2_driver_layer/src/Tac3D-v3.1.3-linux',
            shell=True,
            output='screen'
        ),

        # 启动右传感器 publisher（建议用唯一节点名）
        Node(
            package='haptic',
            executable='tac3d_r',
            name='tac3d_r_node',
            output='screen'
        ),

        # 启动右传感器 publisher
        Node(
            package='haptic',
            executable='tac3d_l',
            name='tac3d_l_node',
            output='screen'
        ),
    ])

