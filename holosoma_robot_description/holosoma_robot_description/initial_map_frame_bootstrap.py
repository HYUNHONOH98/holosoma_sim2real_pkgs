import math

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformException, TransformListener


def _quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class InitialMapFrameBootstrap(Node):
    def __init__(self) -> None:
        super().__init__("initial_map_frame_bootstrap")

        self.declare_parameter("map_frame_id", "map")
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("initial_camera_frame_id", "mid360_link")
        self.declare_parameter("left_foot_frame_id", "LL_FOOT")
        self.declare_parameter("right_foot_frame_id", "LR_FOOT")
        self.declare_parameter("height_mode", "average_abs_z")
        self.declare_parameter("map_to_odom_z_sign", 1.0)
        self.declare_parameter("map_to_odom_roll", 0.0)
        self.declare_parameter("map_to_odom_pitch", 0.0)
        self.declare_parameter("map_to_odom_yaw", 0.0)
        self.declare_parameter("startup_timeout_sec", 10.0)
        self.declare_parameter("fallback_initial_z", 0.0)
        self.declare_parameter("retry_period_sec", 0.1)

        self._map_frame_id = self.get_parameter("map_frame_id").get_parameter_value().string_value
        self._odom_frame_id = self.get_parameter("odom_frame_id").get_parameter_value().string_value
        self._camera_frame_id = self.get_parameter("initial_camera_frame_id").get_parameter_value().string_value
        self._left_foot_frame_id = self.get_parameter("left_foot_frame_id").get_parameter_value().string_value
        self._right_foot_frame_id = self.get_parameter("right_foot_frame_id").get_parameter_value().string_value
        self._height_mode = self.get_parameter("height_mode").get_parameter_value().string_value
        self._z_sign = self.get_parameter("map_to_odom_z_sign").get_parameter_value().double_value
        self._map_to_odom_rpy = (
            self.get_parameter("map_to_odom_roll").get_parameter_value().double_value,
            self.get_parameter("map_to_odom_pitch").get_parameter_value().double_value,
            self.get_parameter("map_to_odom_yaw").get_parameter_value().double_value,
        )
        self._map_to_odom_quat = _quaternion_from_rpy(*self._map_to_odom_rpy)
        self._startup_timeout_sec = self.get_parameter("startup_timeout_sec").get_parameter_value().double_value
        self._fallback_initial_z = self.get_parameter("fallback_initial_z").get_parameter_value().double_value
        retry_period_sec = self.get_parameter("retry_period_sec").get_parameter_value().double_value

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._static_broadcaster = StaticTransformBroadcaster(self)
        self._start_time = self.get_clock().now()
        self._published = False

        self._timer = self.create_timer(max(0.01, retry_period_sec), self._try_publish_transform)

    def _lookup_foot_z(self, foot_frame_id: str) -> float:
        transform = self._tf_buffer.lookup_transform(
            self._camera_frame_id,
            foot_frame_id,
            Time(),
            timeout=Duration(seconds=0.05),
        )
        return float(transform.transform.translation.z)

    def _compute_height(self, left_z: float, right_z: float) -> float:
        if self._height_mode == "average_abs_z":
            return 0.5 * (abs(left_z) + abs(right_z))
        if self._height_mode == "average_z":
            return 0.5 * (left_z + right_z)
        if self._height_mode == "max_abs_z":
            return max(abs(left_z), abs(right_z))
        if self._height_mode == "min_abs_z":
            return min(abs(left_z), abs(right_z))
        raise ValueError("height_mode must be one of: average_abs_z, average_z, max_abs_z, min_abs_z")

    def _try_publish_transform(self) -> None:
        if self._published:
            return

        try:
            left_z = self._lookup_foot_z(self._left_foot_frame_id)
            right_z = self._lookup_foot_z(self._right_foot_frame_id)
            initial_z = self._compute_height(left_z, right_z)
        except (TransformException, ValueError) as exc:
            elapsed_sec = (self.get_clock().now() - self._start_time).nanoseconds * 1e-9
            if self._startup_timeout_sec > 0.0 and elapsed_sec >= self._startup_timeout_sec:
                if math.isfinite(self._fallback_initial_z) and self._fallback_initial_z > 0.0:
                    initial_z = self._fallback_initial_z
                    left_z = float("nan")
                    right_z = float("nan")
                    self.get_logger().warn(f"Using fallback_initial_z={initial_z:.3f}; TF lookup failed: {exc}")
                else:
                    self.get_logger().warn(
                        "Still waiting for initial foot TFs: "
                        f"{self._camera_frame_id} -> [{self._left_foot_frame_id}, {self._right_foot_frame_id}]. "
                        f"Last error: {exc}"
                    )
                    self._start_time = self.get_clock().now()
                    return
            else:
                return

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self._map_frame_id
        transform.child_frame_id = self._odom_frame_id
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = self._z_sign * initial_z
        transform.transform.rotation.x = self._map_to_odom_quat[0]
        transform.transform.rotation.y = self._map_to_odom_quat[1]
        transform.transform.rotation.z = self._map_to_odom_quat[2]
        transform.transform.rotation.w = self._map_to_odom_quat[3]

        self._static_broadcaster.sendTransform(transform)
        self._published = True
        self.get_logger().info(
            "Published static {} -> {} with z={:.3f}, rpy=({:.3f}, {:.3f}, {:.3f}) "
            "from foot z values left={} right={}".format(
                self._map_frame_id,
                self._odom_frame_id,
                transform.transform.translation.z,
                *self._map_to_odom_rpy,
                "nan" if math.isnan(left_z) else "{:.3f}".format(left_z),
                "nan" if math.isnan(right_z) else "{:.3f}".format(right_z),
            )
        )


def main() -> None:
    rclpy.init()
    node = InitialMapFrameBootstrap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
