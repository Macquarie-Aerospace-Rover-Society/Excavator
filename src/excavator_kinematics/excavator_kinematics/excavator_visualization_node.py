import math
from typing import Iterable, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Vector3
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray

from excavator_kinematics.state_codec import decode_state, mm_xy_to_point_xyz


class ExcavatorVisualizationNode(Node):
    def __init__(self) -> None:
        super().__init__("excavator_visualization_node")

        self.declare_parameter("frame_id", "excavator_base")
        self.declare_parameter("state_topic", "excavator/state")

        self.frame_id = str(self.get_parameter("frame_id").value)
        state_topic = str(self.get_parameter("state_topic").value)

        self.state_sub = self.create_subscription(Float32MultiArray, state_topic, self._state_cb, 10)

        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.stroke_pub = self.create_publisher(Float32MultiArray, "excavator/stroke_monitor", 10)
        self.force_pub = self.create_publisher(Vector3, "excavator/forces", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "excavator/markers", 10)

        self.get_logger().info("Excavator visualization node started. State topic: %s" % state_topic)

    def _point_mm(self, xy_mm: Tuple[float, float]) -> Point:
        x, y, z = mm_xy_to_point_xyz(xy_mm)
        pt = Point()
        pt.x = x
        pt.y = y
        pt.z = z
        return pt

    def _make_line_marker(
        self,
        marker_id: int,
        ns: str,
        points_mm: Iterable[Tuple[float, float]],
        color: Tuple[float, float, float, float],
        width_m: float,
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = width_m
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        marker.points = [self._point_mm(pt) for pt in points_mm]
        return marker

    def _state_cb(self, msg: Float32MultiArray) -> None:
        try:
            state = decode_state(msg)
        except ValueError as exc:
            self.get_logger().warning(str(exc))
            return

        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = ["excavator_arm_joint", "excavator_bucket_joint"]
        bucket_rel_deg = state.rt_deg - state.re_deg
        joint_msg.position = [math.radians(state.re_deg), math.radians(bucket_rel_deg)]
        self.joint_pub.publish(joint_msg)

        stroke_msg = Float32MultiArray()
        stroke_msg.data = [
            float(state.l1_mm),
            float(state.l2_mm),
            float(state.l1_pct),
            float(state.l2_pct),
            float(state.re_dot),
            float(state.rt_dot),
        ]
        self.stroke_pub.publish(stroke_msg)

        force_msg = Vector3()
        force_msg.x = float(state.f_l1_n)
        force_msg.y = float(state.f_l2_n)
        force_msg.z = float(state.mass_kg)
        self.force_pub.publish(force_msg)

        markers = MarkerArray()
        markers.markers.append(
            self._make_line_marker(
                0,
                "links",
                [state.j3_mm, state.j2_mm],
                (0.95, 0.85, 0.80, 1.0),
                0.02,
            )
        )

        l1_color = (1.0, 0.0, 0.0, 1.0) if state.l1_warn else (1.0, 0.6, 0.6, 1.0)
        l2_color = (1.0, 0.0, 0.0, 1.0) if state.l2_warn else (1.0, 0.8, 0.6, 1.0)

        markers.markers.append(self._make_line_marker(1, "links", [state.j1_mm, state.j2_mm], l1_color, 0.015))
        markers.markers.append(self._make_line_marker(2, "links", [state.j3_mm, state.j4_mm], l2_color, 0.015))
        markers.markers.append(
            self._make_line_marker(
                3,
                "links",
                [state.j2_mm, state.j4_mm],
                (0.10, 0.10, 0.10, 1.0),
                0.01,
            )
        )
        markers.markers.append(
            self._make_line_marker(
                4,
                "bucket",
                [state.j2_mm, state.j4_mm, state.tip_mm, state.j2_mm],
                (0.98, 0.92, 0.75, 1.0),
                0.01,
            )
        )

        joint_marker = Marker()
        joint_marker.header.frame_id = self.frame_id
        joint_marker.header.stamp = self.get_clock().now().to_msg()
        joint_marker.ns = "joints"
        joint_marker.id = 5
        joint_marker.type = Marker.SPHERE_LIST
        joint_marker.action = Marker.ADD
        joint_marker.scale.x = 0.03
        joint_marker.scale.y = 0.03
        joint_marker.scale.z = 0.03
        joint_marker.color.r = 1.0
        joint_marker.color.g = 1.0
        joint_marker.color.b = 1.0
        joint_marker.color.a = 1.0
        joint_marker.points = [
            self._point_mm(state.j1_mm),
            self._point_mm(state.j2_mm),
            self._point_mm(state.j3_mm),
            self._point_mm(state.j4_mm),
        ]
        markers.markers.append(joint_marker)

        mass_marker = Marker()
        mass_marker.header.frame_id = self.frame_id
        mass_marker.header.stamp = self.get_clock().now().to_msg()
        mass_marker.ns = "payload"
        mass_marker.id = 6
        mass_marker.type = Marker.SPHERE
        mass_marker.action = Marker.ADD
        mass_marker.pose.position = self._point_mm(state.j4_mm)
        mass_marker.scale.x = 0.04
        mass_marker.scale.y = 0.04
        mass_marker.scale.z = 0.04
        mass_marker.color.r = 0.9
        mass_marker.color.g = 0.1
        mass_marker.color.b = 0.1
        mass_marker.color.a = 1.0
        markers.markers.append(mass_marker)

        self.marker_pub.publish(markers)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ExcavatorVisualizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
