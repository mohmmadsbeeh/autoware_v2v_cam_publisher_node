from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch.event_handlers import OnStateTransition

def generate_launch_description():
    lifecycle_node = LifecycleNode(
        package='udp_driver',
        executable='udp_sender_node_exe',
        name='udp_sender_node',
        output='screen',
        parameters=['/home/autoware_ws/src/autoware_v2v_cam_publisher_node//udp_driver_params.yaml'],
    )

    # Define events to transition the node through the lifecycle
    configure_event = ChangeState(
        lifecycle_node_matcher=launch.events.matches_action(lifecycle_node),
        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE
    )

    activate_event = ChangeState(
        lifecycle_node_matcher=launch.events.matches_action(lifecycle_node),
        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE
    )

    return LaunchDescription([
        lifecycle_node,
        RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node=lifecycle_node,
                start_state='unconfigured',
                goal_state='inactive',
                entities=[activate_event],
            )
        ),
        RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node=lifecycle_node,
                start_state='inactive',
                goal_state='active',
                entities=[],
            )
        ),
        configure_event,
    ])
