from __future__ import annotations

import xml.etree.ElementTree as ET

import rclpy
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


def _parse_index_list(value: str) -> list[int]:
    text = value.strip()
    if not text:
        return []
    return [int(item.strip()) for item in text.split(",") if item.strip()]


class UnitreeLowStateToJointState(Node):
    """Publish ROS JointState messages from Unitree DDS lowstate."""

    def __init__(self) -> None:
        super().__init__("unitree_lowstate_to_joint_state")

        self.declare_parameter("robot_description", "")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("lowstate_topic", "rt/lowstate")
        self.declare_parameter("message_type", "hg")
        self.declare_parameter("dds_domain_id", 0)
        self.declare_parameter("network_interface", "")
        self.declare_parameter("joint_to_motor_indices", "")

        robot_description = str(self.get_parameter("robot_description").value)
        self._joint_names = _joint_names_from_urdf(robot_description)
        if not self._joint_names:
            raise RuntimeError("robot_description did not contain movable joints")

        joint_to_motor_indices = _parse_index_list(
            str(self.get_parameter("joint_to_motor_indices").value)
        )
        if joint_to_motor_indices:
            if len(joint_to_motor_indices) != len(self._joint_names):
                raise RuntimeError(
                    "joint_to_motor_indices length must match the number of movable URDF joints "
                    f"({len(self._joint_names)}), got {len(joint_to_motor_indices)}"
                )
            self._joint_to_motor_indices = joint_to_motor_indices
        else:
            self._joint_to_motor_indices = list(range(len(self._joint_names)))

        joint_states_topic = str(self.get_parameter("joint_states_topic").value)
        self._publisher = self.create_publisher(JointState, joint_states_topic, 10)
        self._warned_short_lowstate = False

        lowstate_topic = str(self.get_parameter("lowstate_topic").value)
        message_type = str(self.get_parameter("message_type").value).strip().lower()
        self._subscriber = self._create_lowstate_subscriber(lowstate_topic, message_type)

        self.get_logger().info(
            f"Publishing {joint_states_topic} from Unitree DDS {lowstate_topic} "
            f"with {len(self._joint_names)} URDF joints"
        )

    def _create_lowstate_subscriber(self, topic: str, message_type: str):
        try:
            from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
        except Exception as exc:
            raise RuntimeError(
                "unitree_sdk2py is required for Unitree lowstate subscriptions. "
                "Source/install the Unitree SDK2 Python environment before launching this node."
            ) from exc

        if message_type in {"hg", "g1", "h1-2", "humanoid"}:
            from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_ as LowState
        elif message_type in {"go", "go2"}:
            from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_ as LowState
        else:
            raise RuntimeError("message_type must be one of: hg, g1, h1-2, humanoid, go, go2")

        domain_id = int(self.get_parameter("dds_domain_id").value)
        network_interface = str(self.get_parameter("network_interface").value).strip()
        if network_interface:
            ChannelFactoryInitialize(domain_id, network_interface)
        else:
            ChannelFactoryInitialize(domain_id)

        subscriber = ChannelSubscriber(topic, LowState)
        subscriber.Init(self._on_lowstate, 1)
        return subscriber

    def _on_lowstate(self, msg) -> None:
        motor_state = getattr(msg, "motor_state", None)
        if motor_state is None:
            return

        max_motor_index = max(self._joint_to_motor_indices)
        if len(motor_state) <= max_motor_index:
            if not self._warned_short_lowstate:
                self.get_logger().warning(
                    "Received lowstate with too few motor_state entries: "
                    f"need index {max_motor_index}, got {len(motor_state)} entries"
                )
                self._warned_short_lowstate = True
            return

        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = list(self._joint_names)

        positions: list[float] = []
        velocities: list[float] = []
        efforts: list[float] = []
        for motor_index in self._joint_to_motor_indices:
            motor = motor_state[motor_index]
            positions.append(float(getattr(motor, "q", 0.0)))
            velocities.append(float(getattr(motor, "dq", 0.0)))
            efforts.append(float(getattr(motor, "tau_est", 0.0)))

        joint_state.position = positions
        joint_state.velocity = velocities
        joint_state.effort = efforts
        self._publisher.publish(joint_state)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = UnitreeLowStateToJointState()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
