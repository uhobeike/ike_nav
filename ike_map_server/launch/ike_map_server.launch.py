import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    ike_map_server_dir = get_package_share_directory('ike_map_server')
    params_file = os.path.join(
        ike_map_server_dir, 'config', 'ike_map_server.param.yaml')

    container = Node(
        name='ike_map_server_container',
        package='rclcpp_components',
        executable='component_container',
        output='both',
    )

    load_composable_nodes = LoadComposableNodes(
        target_container='ike_map_server_container',
        composable_node_descriptions=[
            ComposableNode(
                package='ike_map_server',
                plugin='ike_nav::IkeMapServer',
                name='map_server',
                parameters=[params_file],
                extra_arguments=[{'use_intra_process_comms': True}],
            )
        ],
    )

    return LaunchDescription([container, load_composable_nodes])
