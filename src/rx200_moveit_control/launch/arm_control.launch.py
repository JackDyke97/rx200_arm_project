from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    start_arg = DeclareLaunchArgument(
        'start', default_value='[0.25, 0.0, 0.30]',
        description='Start position (x, y, z)'
    )
    pick_arg = DeclareLaunchArgument(
        'pick', default_value='[0.25, 0.0, 0.18]',
        description='Pick position (x, y, z)'
    )
    place_arg = DeclareLaunchArgument(
        'place', default_value='[-0.20, 0.0, 0.18]',
        description='Place position (x, y, z)'
    )

    arm_node = Node(
        package='rx200_moveit_control',
        executable='rx200_moveit_control',
        output='screen',
        parameters=[
            {'start': LaunchConfiguration('start')},
            {'pick': LaunchConfiguration('pick')},
            {'place': LaunchConfiguration('place')}
        ]
    )

    gripper_node = Node(
        package='rx200_moveit_control',
        executable='rx200_gripper_control',
        output='screen'
    )

    return LaunchDescription([
        start_arg,
        pick_arg,
        place_arg,
        arm_node,
        gripper_node
    ])
