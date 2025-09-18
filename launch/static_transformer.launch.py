from launch import LaunchContext, LaunchDescription

from launch.actions import OpaqueFunction
from launch.substitutions import EnvironmentVariable, TextSubstitution

from launch_ros.actions import Node


def launch_setup(context: LaunchContext):
    uav_name = EnvironmentVariable('UAV_NAME').perform(context)

    fcu_frame = uav_name + '/fcu'
    fcu_frame_slashless = 'fcu_' + uav_name

    fast_lio_frame = uav_name + '/fast_lio'
    fast_lio_frame_slashless = uav_name + '_fast_lio'

    # Declare nodes
    fcu_to_fast_lio_tf_static_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=TextSubstitution(text=fcu_frame_slashless + '_to_' + fast_lio_frame_slashless),
        namespace=TextSubstitution(text=uav_name),
        output='screen',
        arguments=['--x', '0.0',
                   '--y', '0.0',
                   '--z', '0.0',
                   '--yaw', '-0.0',
                   '--pitch', '0.0',
                   '--roll', '-0.0',
                   '--frame-id', fcu_frame,
                   '--child-frame-id', fast_lio_frame])

    return [fcu_to_fast_lio_tf_static_publisher_node]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
