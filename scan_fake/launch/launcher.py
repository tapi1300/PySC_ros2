from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    fpub = Node(
        package='scan_fake',
        executable='node_pub',
        output='screen')

    fsub = Node(
        package='scan_fake',
        executable='node_sub',
        output='screen')

    ld = LaunchDescription()
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(fpub)
    ld.add_action(fsub)
    return ld