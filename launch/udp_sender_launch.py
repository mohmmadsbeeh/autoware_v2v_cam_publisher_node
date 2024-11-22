from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
import launch
import lifecycle_msgs.msg
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the path to the parameter file
    udp_driver_share = get_package_share_directory('udp_driver')
    param_file = os.path.join(udp_driver_share, 'config', 'udp_sender_node.yaml')

    # Create the lifecycle node
    udp_sender_node = LifecycleNode(
        package='udp_driver',
        executable='udp_sender_node_exe',
        name='udp_sender_node',
        namespace='',
        output='screen',
        parameters=['/home/autoware_ws/src/transport_drivers/udp_driver/params/udp_driver_params.yaml'],
    )

    # Emit an event to configure the node
    configure_transition = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(udp_sender_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE
        )
    )

    # Emit an event to activate the node after it becomes inactive
    activate_event_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=udp_sender_node,
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=launch.events.matches_action(udp_sender_node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE
                    )
                )
            ],
        )
    )

    return LaunchDescription([
        udp_sender_node,
        configure_transition,
        activate_event_handler,
    ])
