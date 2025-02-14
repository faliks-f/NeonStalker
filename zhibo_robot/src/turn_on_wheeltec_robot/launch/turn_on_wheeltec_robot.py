import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'usart_port_name', default_value='/dev/wheeltec_controller', description='USART Port Name'
        ),
        DeclareLaunchArgument(
            'serial_baud_rate', default_value='115200', description='Serial Baud Rate'
        ),
        DeclareLaunchArgument(
            'odom_frame_id', default_value='odom_combined', description='Frame ID for odometry'
        ),
        DeclareLaunchArgument(
            'robot_frame_id', default_value='base_footprint', description='Frame ID for robot base'
        ),
        DeclareLaunchArgument(
            'gyro_frame_id', default_value='gyro_link', description='Frame ID for gyro sensor'
        ),
        DeclareLaunchArgument(
            'odom_x_scale', default_value='1.0', description='X scale for odometry'
        ),
        DeclareLaunchArgument(
            'odom_y_scale', default_value='1.0', description='Y scale for odometry'
        ),
        DeclareLaunchArgument(
            'odom_z_scale_positive', default_value='1.0', description='Positive Z scale for odometry'
        ),
        DeclareLaunchArgument(
            'odom_z_scale_negative', default_value='1.0', description='Negative Z scale for odometry'
        ),

        # Launch the node and set parameters dynamically
        Node(
            package='turn_on_wheeltec_robot',
            executable='wheeltec_robot_node',
            name='wheeltec_robot_node',
            parameters=[{
                'usart_port_name': LaunchConfiguration('usart_port_name'),
                'serial_baud_rate': LaunchConfiguration('serial_baud_rate'),
                'odom_frame_id': LaunchConfiguration('odom_frame_id'),
                'robot_frame_id': LaunchConfiguration('robot_frame_id'),
                'gyro_frame_id': LaunchConfiguration('gyro_frame_id'),
                'odom_x_scale': LaunchConfiguration('odom_x_scale'),
                'odom_y_scale': LaunchConfiguration('odom_y_scale'),
                'odom_z_scale_positive': LaunchConfiguration('odom_z_scale_positive'),
                'odom_z_scale_negative': LaunchConfiguration('odom_z_scale_negative')
            }],
            output='screen'
        )
    ])
