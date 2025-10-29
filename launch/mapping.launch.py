import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch.conditions import IfCondition

from launch_ros.actions import Node


def generate_launch_description():
    package_path = get_package_share_directory('fast_lio')
    default_config_path = os.path.join(package_path, 'config')

    declared_namespace = DeclareLaunchArgument(
        'namespace',
        default_value=EnvironmentVariable('UAV_NAME'),
        description='Namespace do nó.'
    )

    namespace = LaunchConfiguration('namespace')

    use_sim_time = LaunchConfiguration('use_sim_time')
    config_file_path = LaunchConfiguration('config_file_path')
    topic_pcl = LaunchConfiguration('topic_pcl')
    topic_imu = LaunchConfiguration('topic_imu')

    declared_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    declared_config_file_path = DeclareLaunchArgument(
        'config_file_path', default_value='',
        description='Yaml config file path'
    )

    declared_topic_imu = 
        DeclareLaunchArgument(
            'topic_imu',
            default_value=['/', os.getenv('UAV_NAME', "uav1"), '/livox/imu'],
            description='Name of the IMU topic.'
    )

    declared_topic_pcl = DeclareLaunchArgument(
            'topic_pcl',
            default_value=['/', os.getenv('UAV_NAME', "uav1"), '/livox/lidar'],
            description='Name of the pcl topic.'
    )


    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        namespace=namespace,
        parameters=[config_file_path, {'use_sim_time': use_sim_time}],
        remappings=[
            ('lidar_in', topic_pcl),
            ('imu_in', topic_imu),
            ('imu_out', 'fast_lio/imu/out'),
            ('cloud_registered_body_out', 'fast_lio/cloud_registered_body'),
            ('cloud_registered_out', 'fast_lio/cloud_registered'),
            ('cloud_effected_out', 'fast_lio/cloud_effected'),
            ('laser_map_out', 'fast_lio/laser_map'),
            ('odometry_out', 'fast_lio/odometry'),
            ('odometry_high_freq_out', 'fast_lio/odometry_high_freq'),
            ('path_out', 'fast_lio/path'),
        ],
        output='screen'
    )


    ld = LaunchDescription()
    
    # 3. Adiciona a declaração do argumento ao LaunchDescription
    ld.add_action(declared_namespace)
    
    ld.add_action(declared_use_sim_time)
    ld.add_action(declared_config_file_path)

    ld.add_action(declared_topic_pcl)
    ld.add_action(declared_topic_imu)

    ld.add_action(fast_lio_node)

    return ld
