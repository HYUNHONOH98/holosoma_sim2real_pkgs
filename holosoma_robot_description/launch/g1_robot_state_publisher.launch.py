from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _launch_setup(context: LaunchContext, *args, **kwargs):
    urdf_path = Path(LaunchConfiguration("urdf").perform(context))
    robot_description = urdf_path.read_text(encoding="utf-8")
    use_sim_time = _as_bool(LaunchConfiguration("use_sim_time").perform(context))
    publish_frequency = float(LaunchConfiguration("publish_frequency").perform(context))
    joint_state_source = LaunchConfiguration("joint_state_source").perform(context).strip().lower()
    robot_state_topic = LaunchConfiguration("robot_state_topic").perform(context)
    joint_states_topic = LaunchConfiguration("joint_states_topic").perform(context)
    lowstate_topic = LaunchConfiguration("lowstate_topic").perform(context)
    unitree_message_type = LaunchConfiguration("unitree_message_type").perform(context)
    unitree_dds_domain_id = int(LaunchConfiguration("unitree_dds_domain_id").perform(context))
    unitree_network_interface = LaunchConfiguration("unitree_network_interface").perform(context)
    joint_to_motor_indices = LaunchConfiguration("joint_to_motor_indices").perform(context)
    use_robot_state_to_joint_state = _as_bool(
        LaunchConfiguration("use_robot_state_to_joint_state").perform(context)
    )
    use_joint_state_publisher = _as_bool(
        LaunchConfiguration("use_joint_state_publisher").perform(context)
    )

    if use_robot_state_to_joint_state:
        joint_state_source = "robot_state"
    if use_joint_state_publisher:
        joint_state_source = "dummy"

    if joint_state_source not in {"lowstate", "robot_state", "dummy", "none"}:
        raise RuntimeError("joint_state_source must be one of: lowstate, robot_state, dummy, none")

    nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[
                {
                    "robot_description": robot_description,
                    "use_sim_time": use_sim_time,
                    "publish_frequency": publish_frequency,
                }
            ],
            remappings=[("joint_states", joint_states_topic)],
        )
    ]

    if joint_state_source == "robot_state":
        nodes.insert(
            0,
            Node(
                package="holosoma_robot_description",
                executable="robot_state_to_joint_state",
                name="robot_state_to_joint_state",
                output="screen",
                parameters=[
                    {
                        "robot_description": robot_description,
                        "robot_state_topic": robot_state_topic,
                        "joint_states_topic": joint_states_topic,
                        "use_sim_time": use_sim_time,
                    }
                ],
            ),
        )

    if joint_state_source == "lowstate":
        nodes.insert(
            0,
            Node(
                package="holosoma_robot_description",
                executable="unitree_lowstate_to_joint_state",
                name="unitree_lowstate_to_joint_state",
                output="screen",
                parameters=[
                    {
                        "robot_description": robot_description,
                        "joint_states_topic": joint_states_topic,
                        "lowstate_topic": lowstate_topic,
                        "message_type": unitree_message_type,
                        "dds_domain_id": unitree_dds_domain_id,
                        "network_interface": unitree_network_interface,
                        "joint_to_motor_indices": joint_to_motor_indices,
                        "use_sim_time": use_sim_time,
                    }
                ],
            ),
        )

    if joint_state_source == "dummy":
        nodes.insert(
            0,
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
                output="screen",
                parameters=[
                    {
                        "robot_description": robot_description,
                        "use_sim_time": use_sim_time,
                    }
                ],
                remappings=[("joint_states", joint_states_topic)],
            ),
        )

    return nodes


def generate_launch_description():
    default_urdf = str(
        Path(get_package_share_directory("holosoma_robot_description"))
        / "urdf"
        / "g1_29dof.urdf"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("urdf", default_value=default_urdf),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("publish_frequency", default_value="200.0"),
            DeclareLaunchArgument("joint_state_source", default_value="none"),
            DeclareLaunchArgument("robot_state_topic", default_value="/robot_state"),
            DeclareLaunchArgument("joint_states_topic", default_value="/joint_states"),
            DeclareLaunchArgument("lowstate_topic", default_value="rt/lowstate"),
            DeclareLaunchArgument("unitree_message_type", default_value="hg"),
            DeclareLaunchArgument("unitree_dds_domain_id", default_value="0"),
            DeclareLaunchArgument("unitree_network_interface", default_value=""),
            DeclareLaunchArgument("joint_to_motor_indices", default_value=""),
            DeclareLaunchArgument("use_robot_state_to_joint_state", default_value="false"),
            DeclareLaunchArgument("use_joint_state_publisher", default_value="false"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
