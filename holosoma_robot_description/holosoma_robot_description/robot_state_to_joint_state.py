from __future__ import annotations

import xml.etree.ElementTree as ET

import rclpy
from far_msgs.msg import RobotState
from rclpy.node import Node
from sensor_msgs.msg import JointState


def _joint_names_from_urdf(urdf_text: str) -> list[str]:
    root = ET.fromstring(urdf_text)
    names: list[str] = []
    for joint in root.findall("joint"):
        if joint.attrib.get("type") == "fixed":
            continue
        name = joint.attrib.get("name")
        if name:
            names.append(name)
    return names


class RobotStateToJointState(Node):
    """Republish far_msgs/RobotState joints as sensor_msgs/JointState."""

    def __init__(self) -> None:
        super().__init__("robot_state_to_joint_state")

        self.declare_parameter("robot_description", "")
        self.declare_parameter("robot_state_topic", "/robot_state")
        self.declare_parameter("joint_states_topic", "/joint_states")

        robot_description = str(self.get_parameter("robot_description").value)
        self._joint_names = _joint_names_from_urdf(robot_description)
        if not self._joint_names:
            raise RuntimeError("robot_description did not contain movable joints")

        robot_state_topic = str(self.get_parameter("robot_state_topic").value)
        joint_states_topic = str(self.get_parameter("joint_states_topic").value)
        self._publisher = self.create_publisher(JointState, joint_states_topic, 10)
        self._subscription = self.create_subscription(
            RobotState,
            robot_state_topic,
            self._on_robot_state,
            10,
        )
        self.get_logger().info(
            f"Publishing {joint_states_topic} from {robot_state_topic} "
            f"with {len(self._joint_names)} URDF joints"
        )

    def _on_robot_state(self, msg: RobotState) -> None:
        position_count = len(msg.joint_positions)
        joint_count = min(len(self._joint_names), position_count)
        if joint_count <= 0:
            return

        joint_state = JointState()
        joint_state.header = msg.header
        if joint_state.header.stamp.sec == 0 and joint_state.header.stamp.nanosec == 0:
            joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = self._joint_names[:joint_count]
        joint_state.position = [float(value) for value in msg.joint_positions[:joint_count]]

        if len(msg.joint_velocities) >= joint_count:
            joint_state.velocity = [float(value) for value in msg.joint_velocities[:joint_count]]
        if len(msg.joint_torques) >= joint_count:
            joint_state.effort = [float(value) for value in msg.joint_torques[:joint_count]]

        self._publisher.publish(joint_state)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = RobotStateToJointState()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
