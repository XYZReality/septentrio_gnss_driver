import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node
from datetime import datetime
from launch.substitutions import LaunchConfiguration

os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '{time}: [{name}] [{severity}]\t{message}'
# Verbose log:
#os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '{time}: [{name}] [{severity}]\t{message} ({function_name}() at {file_name}:{line_number})'

def generate_launch_description():
    ld = LaunchDescription()

    package_dir = get_package_share_directory('septentrio_gnss_driver')
    config_path = os.path.join(package_dir, 'config', 'jetson.yaml')

    ld.add_action(DeclareLaunchArgument(
        'gnss_config', default_value=config_path, description='Path to the config file'
    ))

    ld.add_action(DeclareLaunchArgument(
        'output_path', default_value='/apps/output', description='Path to the output folder'
    ))

    node = Node(
            package='septentrio_gnss_driver',
            executable='septentrio_gnss_driver_node',
            name='septentrio_gnss_driver',
            emulate_tty=True,
            sigterm_timeout = '20',
            parameters=[LaunchConfiguration('gnss_config'),
                {'output_path': LaunchConfiguration('output_path')}])

    ld.add_action(node)
    return ld