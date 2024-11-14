# cam_publisher_launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='autoware_v2x_cam_publisher',
            executable='cam_publisher_node',
            name='cam_publisher_node',
            parameters=[
                {'station_id': 1},  # Station ID

                # Vehicle dimensions and parameters
                {'wheel_radius': 0.3},
                {'wheel_width': 0.2},
                {'wheel_base': 2.5},
                {'wheel_tread': 1.6},
                {'front_overhang': 0.8},
                {'rear_overhang': 0.8},
                {'left_overhang': 0.5},
                {'right_overhang': 0.5},
                {'vehicle_height': 1.5},
                {'vehicle_width': 1.8},
                {'vehicle_length': 4.1},  # wheel_base + front_overhang + rear_overhang
                {'max_steer_angle': 0.785},  # in radians (~45 degrees)

                # Reference position for coordinate conversion
                {'reference_latitude': 0.0},
                {'reference_longitude': 0.0},
                {'reference_altitude': 0.0},
            ],
            output='screen'
        )
    ])
