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
    default_rviz_config_path = os.path.join(
        package_path, 'rviz', 'fastlio.rviz')

    # --- INÍCIO DAS ALTERAÇÕES ---

    # 1. Declaração do argumento de namespace
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value=EnvironmentVariable('UAV_NAME'),
        description='Namespace do nó.'
    )

    # Inicializa a configuração do namespace
    namespace = LaunchConfiguration('namespace')

    # --- FIM DAS ALTERAÇÕES ---

    use_sim_time = LaunchConfiguration('use_sim_time')
    config_path = LaunchConfiguration('config_path')
    config_file = LaunchConfiguration('config_file')
    rviz_use = LaunchConfiguration('rviz')
    rviz_cfg = LaunchConfiguration('rviz_cfg')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_config_path_cmd = DeclareLaunchArgument(
        'config_path', default_value=default_config_path,
        description='Yaml config file path'
    )
    decalre_config_file_cmd = DeclareLaunchArgument(
        'config_file', default_value='mid360.yaml',
        description='Config file'
    )
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Use RViz to monitor results'
    )
    declare_rviz_config_path_cmd = DeclareLaunchArgument(
        'rviz_cfg', default_value=default_rviz_config_path,
        description='RViz config file path'
    )

    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        # 2. Adição do namespace ao nó
        namespace=namespace,
        parameters=[PathJoinSubstitution([config_path, config_file]),
                    {'use_sim_time': use_sim_time}],
        remappings=[
            ('lidar_in', 'livox/points'),
            ('imu_in', 'livox/imu'),
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
    ld.add_action(declare_namespace_cmd)
    
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_config_path_cmd)
    ld.add_action(decalre_config_file_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_rviz_config_path_cmd)

    ld.add_action(fast_lio_node)

    return ld