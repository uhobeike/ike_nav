from launch import LaunchDescription
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    container = Node(
        name='ike_planner_container',
        package='rclcpp_components',
        executable='component_container',
        output='both',
    )

    load_composable_nodes = LoadComposableNodes(
        target_container='ike_planner_container',
        composable_node_descriptions=[
            ComposableNode(
                package='ike_planner',
                plugin='ike_nav::IkePlanner',
                name='planner',
                extra_arguments=[{'use_intra_process_comms': True}],
            )
        ],
    )

    return LaunchDescription([container, load_composable_nodes])
