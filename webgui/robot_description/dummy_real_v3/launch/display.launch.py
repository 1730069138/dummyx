import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'dummy_real_v3'
    pkg_share = get_package_share_directory(package_name)
    urdf_file = os.path.join(pkg_share, 'urdf', 'dummy_real_v3.urdf')
    rviz_config_file = os.path.join(pkg_share, 'urdf.rviz')

    # 读取 URDF 内容
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # 如果原工程存在 RViz 配置文件则使用，否则空载启动
    rviz_args = ['-d', rviz_config_file] if os.path.exists(rviz_config_file) else []

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=rviz_args
        )
    ])
