import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 获取包路径
    target_model_pkg_share = get_package_share_directory('target_model_pkg')
    
    # 1. 启动带ROS插件的Gazebo
    gazebo_ros_share = get_package_share_directory('gazebo_ros')
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': 'empty.world'}.items()
    )

    # 2. 定义参数
    model = LaunchConfiguration('model', default=os.path.join(
        target_model_pkg_share, 'urdf/armor/armor_1.sdf'))
    model_name = LaunchConfiguration('model_name', default='armor')

    # 3. 生成实体
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-file', model,
            '-entity', model_name,
            '-x', '0.0', '-y', '1.0', '-z', '0.5', '-R', '0.0'
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,
        spawn_entity
    ])
