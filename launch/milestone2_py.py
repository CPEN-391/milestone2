from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('f1tenth_gym_ros'),
        'config',
        'sim.yaml'
    )

    config_dict = yaml.safe_load(open(config, 'r'))

    bridge_node = Node(
        package='f1tenth_gym_ros',
        executable='gym_bridge',
        name='bridge',
        parameters=[config]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', os.path.join(
            get_package_share_directory('f1tenth_gym_ros'),
            'launch',
            'gym_bridge.rviz')]
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        parameters=[
            {'yaml_filename': config_dict['bridge']['ros__parameters']['map_path'] + '.yaml'},
            {'topic': 'map'},
            {'frame_id': 'map'},
            {'output': 'screen'},
            {'use_sim_time': True}
        ]
    )

    nav_lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'autostart': True},
            {'node_names': ['map_server']}
        ]
    )

    ego_robot_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='ego_robot_state_publisher',
        parameters=[{
            'robot_description': Command([
                'xacro ',
                os.path.join(
                    get_package_share_directory('f1tenth_gym_ros'),
                    'launch',
                    'ego_racecar.xacro'
                )
            ])
        }],
        remappings=[('/robot_description', 'ego_robot_description')]
    )

    milestone2_safety_node = Node(
        package='milestone2',
        executable='safety_node',
        name='milestone2_safety_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    milestone2_wall_follow_node = Node(
        package='milestone2',
        executable='wall_follow_node',
        name='milestone2_wall_follow_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    milestone2_gap_follow_node = Node(
        package='milestone2',
        executable='gap_follow_node',
        name='milestone2_gap_follow_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # finalize
    ld.add_action(rviz_node)
    ld.add_action(bridge_node)
    ld.add_action(nav_lifecycle_node)
    ld.add_action(map_server_node)
    ld.add_action(ego_robot_publisher)
    ld.add_action(milestone2_safety_node)
    #ld.add_action(milestone2_wall_follow_node)
    ld.add_action(milestone2_gap_follow_node)

    return ld
