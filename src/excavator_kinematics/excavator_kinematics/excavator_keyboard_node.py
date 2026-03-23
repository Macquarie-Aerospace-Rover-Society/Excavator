import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3


HELP_TEXT = """
Excavator keyboard control:
  w/s : increase/decrease elevation re (deg)
  a/d : increase/decrease tilt rt (deg)
  r/f : increase/decrease load mass (kg)
  z   : reset to initial setpoints
  q   : quit keyboard node
""".strip()


class ExcavatorKeyboardNode(Node):
    def __init__(self) -> None:
        super().__init__("excavator_keyboard_node")

        self.declare_parameter("command_topic", "excavator/command")
        self.declare_parameter("step_re_deg", 2.0)
        self.declare_parameter("step_rt_deg", 2.0)
        self.declare_parameter("step_mass_kg", 1.0)
        self.declare_parameter("initial_re_deg", 30.0)
        self.declare_parameter("initial_rt_deg", -90.0)
        self.declare_parameter("initial_mass_kg", 10.0)

        command_topic = str(self.get_parameter("command_topic").value)
        self.step_re = float(self.get_parameter("step_re_deg").value)
        self.step_rt = float(self.get_parameter("step_rt_deg").value)
        self.step_mass = float(self.get_parameter("step_mass_kg").value)

        self.re_deg = float(self.get_parameter("initial_re_deg").value)
        self.rt_deg = float(self.get_parameter("initial_rt_deg").value)
        self.mass_kg = float(self.get_parameter("initial_mass_kg").value)

        self.initial_re = self.re_deg
        self.initial_rt = self.rt_deg
        self.initial_mass = self.mass_kg

        self.pub = self.create_publisher(Vector3, command_topic, 10)
        self.timer = self.create_timer(0.05, self._poll_keyboard)

        self._stdin_fd = None
        self._term_settings = None
        self._tty_enabled = False

        self._enable_raw_tty()
        self._publish_command()

        self.get_logger().info("Keyboard node started. Publishing to %s" % command_topic)
        self.get_logger().info("\n" + HELP_TEXT)

    def _enable_raw_tty(self) -> None:
        if not sys.stdin.isatty():
            self.get_logger().warning("stdin is not a TTY; keyboard control disabled for this node.")
            return

        self._stdin_fd = sys.stdin.fileno()
        self._term_settings = termios.tcgetattr(self._stdin_fd)
        tty.setcbreak(self._stdin_fd)
        self._tty_enabled = True

    def _restore_tty(self) -> None:
        if self._tty_enabled and self._stdin_fd is not None and self._term_settings is not None:
            termios.tcsetattr(self._stdin_fd, termios.TCSADRAIN, self._term_settings)
        self._tty_enabled = False

    def _publish_command(self) -> None:
        msg = Vector3()
        msg.x = self.re_deg
        msg.y = self.rt_deg
        msg.z = self.mass_kg
        self.pub.publish(msg)

    def _poll_keyboard(self) -> None:
        if not self._tty_enabled or self._stdin_fd is None:
            return

        ready, _, _ = select.select([sys.stdin], [], [], 0.0)
        if not ready:
            return

        ch = sys.stdin.read(1)
        updated = False

        if ch == "w":
            self.re_deg += self.step_re
            updated = True
        elif ch == "s":
            self.re_deg -= self.step_re
            updated = True
        elif ch == "a":
            self.rt_deg += self.step_rt
            updated = True
        elif ch == "d":
            self.rt_deg -= self.step_rt
            updated = True
        elif ch == "r":
            self.mass_kg += self.step_mass
            updated = True
        elif ch == "f":
            self.mass_kg = max(0.0, self.mass_kg - self.step_mass)
            updated = True
        elif ch == "z":
            self.re_deg = self.initial_re
            self.rt_deg = self.initial_rt
            self.mass_kg = self.initial_mass
            updated = True
        elif ch == "q":
            self.get_logger().info("Keyboard node exiting on user request.")
            rclpy.shutdown()
            return

        if updated:
            self._publish_command()
            self.get_logger().info(
                "cmd re=%.1f deg, rt=%.1f deg, mass=%.1f kg" % (self.re_deg, self.rt_deg, self.mass_kg)
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ExcavatorKeyboardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._restore_tty()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
