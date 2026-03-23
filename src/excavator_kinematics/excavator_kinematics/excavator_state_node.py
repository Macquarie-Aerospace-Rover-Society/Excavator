import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32MultiArray

from excavator_kinematics.kinematics_model import ExcavatorConfig, ExcavatorMechanism
from excavator_kinematics.state_codec import encode_state


class ExcavatorStateNode(Node):
    def __init__(self) -> None:
        super().__init__("excavator_state_node")

        self.declare_parameter("s1_mm", 505.0)
        self.declare_parameter("c_mm", 125.0)
        self.declare_parameter("b_mm", 125.0)
        self.declare_parameter("actuator_min_mm", 380.0)
        self.declare_parameter("actuator_max_mm", 630.0)
        self.declare_parameter("warn_thresh_mm", 5.0)
        self.declare_parameter("v_stroke_mmps", 5.0)
        self.declare_parameter("gravity_mps2", 9.81)

        self.declare_parameter("initial_re_deg", 30.0)
        self.declare_parameter("initial_rt_deg", -90.0)
        self.declare_parameter("initial_mass_kg", 10.0)

        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("command_topic", "excavator/command")
        self.declare_parameter("state_topic", "excavator/state")

        cfg = ExcavatorConfig(
            s1_mm=float(self.get_parameter("s1_mm").value),
            c_mm=float(self.get_parameter("c_mm").value),
            b_mm=float(self.get_parameter("b_mm").value),
            actuator_min_mm=float(self.get_parameter("actuator_min_mm").value),
            actuator_max_mm=float(self.get_parameter("actuator_max_mm").value),
            warn_thresh_mm=float(self.get_parameter("warn_thresh_mm").value),
            v_stroke_mmps=float(self.get_parameter("v_stroke_mmps").value),
            gravity_mps2=float(self.get_parameter("gravity_mps2").value),
        )
        self.model = ExcavatorMechanism(cfg)

        self.current_re = float(self.get_parameter("initial_re_deg").value)
        self.current_rt = float(self.get_parameter("initial_rt_deg").value)
        self.current_mass = float(self.get_parameter("initial_mass_kg").value)
        self.current_c = cfg.c_mm
        self.current_b = cfg.b_mm

        command_topic = str(self.get_parameter("command_topic").value)
        state_topic = str(self.get_parameter("state_topic").value)

        self.command_sub = self.create_subscription(Vector3, command_topic, self._command_cb, 10)
        self.state_pub = self.create_publisher(Float32MultiArray, state_topic, 10)

        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.timer = self.create_timer(1.0 / max(1.0, publish_rate_hz), self._on_timer)

        self.get_logger().info("Excavator state node started. Command topic: %s" % command_topic)

    def _command_cb(self, msg: Vector3) -> None:
        self.current_re = msg.x
        self.current_rt = msg.y
        self.current_mass = max(msg.z, 0.0)

    def _on_timer(self) -> None:
        state = self.model.compute(
            re_deg=self.current_re,
            rt_deg=self.current_rt,
            mass_kg=self.current_mass,
            c_mm=self.current_c,
            b_mm=self.current_b,
        )
        self.current_re = state.re_deg
        self.current_rt = state.rt_deg
        self.state_pub.publish(encode_state(state))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ExcavatorStateNode()
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
