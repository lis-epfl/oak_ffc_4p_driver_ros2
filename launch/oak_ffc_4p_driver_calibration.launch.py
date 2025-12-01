from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    ld = LaunchDescription()
    # Create the NatNet client node
    config = os.path.join(
        get_package_share_directory('oak_ffc_4p_driver_ros2'),
        'config',
        'cam_calibration_config.yaml'
    )
    oak_ffc_4p_driver_node = Node(
        package='oak_ffc_4p_driver_ros2',
        executable='oak_ffc_4p_driver_node',
        name='oak_ffc_4p_driver_node',
        parameters=[config],
        # prefix=['xterm -fa default -fs 10 -e gdb -ex run --args'],
        output='screen',
        emulate_tty=True
    )

    ld.add_action(oak_ffc_4p_driver_node)
    return ld
