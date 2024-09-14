import launch
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription

use_socketcan = LaunchConfiguration('use_socketcan')

def generate_launch_description():

    declare_use_socketcan_cmd = DeclareLaunchArgument(
        name='use_socketcan',
        default_value='True',
        description='use socketcan if true')

    socketcan_send_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros2_socketcan"),
                "launch/socket_can_sender.launch.py"
            )
        ),
        condition=launch.conditions.IfCondition(use_socketcan),
        launch_arguments={'interface': 'can0', 'interval_sec' : '0.01', 'enable_can_fd' : 'False'}.items()
    )

    socketcan_receive_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros2_socketcan"),
                "launch/socket_can_receiver.launch.py"
            )
        ),
        condition=launch.conditions.IfCondition(use_socketcan),
        launch_arguments={'interface' : 'can0', 'timeout_sec' : '0.01', 'enable_can_fd' : 'False'}.items()
    )

    servo_node = Node(
            package='ros2_keya_servo',
            executable='ros2_keya_servo',
            name='ros2_keya_servo',
            output='screen')

    return LaunchDescription([
        declare_use_socketcan_cmd,
        socketcan_send_launch,
        socketcan_receive_launch,
        servo_node,
    ])