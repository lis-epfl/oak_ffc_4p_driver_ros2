from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()

    # USB speed override — drones with marginal SuperSpeed Plus signal
    # integrity (bad cable / connector) can fall back to "super" via
    # `usb_speed:=super` from a parent launch.  Default is "super_plus".
    ld.add_action(DeclareLaunchArgument(
        'usb_speed',
        default_value='super_plus',
        description='OAK USB speed cap: super_plus (10Gbps), super (5Gbps), high (480Mbps)',
    ))

    config = os.path.join(
        get_package_share_directory('oak_ffc_4p_driver_ros2'),
        'config',
        'cam_omninxt_config.yaml',
    )
    oak_ffc_4p_driver_node = Node(
        package='oak_ffc_4p_driver_ros2',
        executable='oak_ffc_4p_driver_node',
        name='oak_ffc_4p_driver_node',
        parameters=[
            config,
            {'usb_speed': LaunchConfiguration('usb_speed')},
        ],
        # prefix=['xterm -fa default -fs 10 -e gdb -ex run --args'],
        output='screen',
        emulate_tty=True,
    )

    ld.add_action(oak_ffc_4p_driver_node)
    return ld
