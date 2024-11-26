from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_share = get_package_share_directory('autoware_v2x_cam_publisher')
    param_file = os.path.join(package_share, 'config', 'cam_publisher_with_mqtt_params.yaml')

    cam_publisher_node_withMQTT = Node(
        package='autoware_v2x_cam_publisher',
        executable='cam_publisher_node_withMQTT',
        name='cam_publisher_node_withMQTT',
        parameters=['autoware_ws/src/autoware_v2x_cam_publisher/config/cam_publisher_with_mqtt_params.yaml'],
        output='screen',
    )

    return LaunchDescription([
        cam_publisher_node_withMQTT,
    ])
