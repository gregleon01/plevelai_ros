import argparse
import time
from typing import Optional

def main(argv: Optional[list[str]] = None) -> None:
    parser = argparse.ArgumentParser(description="Fire controller stub")
    parser.add_argument('--use_sim_time', action='store_true', help='Use simulated time')
    parser.add_argument('--safety_window_ms', type=int, default=500)
    args = parser.parse_args(argv)

    try:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import Bool
        from geometry_msgs.msg import Vector2
        from plevelai_interfaces.msg import WeedDetection
    except Exception:  # pragma: no cover
        print("[fire_controller] ROS 2 not available; stub exit.")
        return

    class FireController(Node):
        def __init__(self) -> None:
            super().__init__('fire_controller')
            self.declare_parameter('use_sim_time', args.use_sim_time)
            self.declare_parameter('safety_window_ms', args.safety_window_ms)
            self.last_aim_time = 0.0
            self.sub_aim = self.create_subscription(Vector2, '/plevelai/aim_cmd', self.on_aim, 10)
            self.sub_det = self.create_subscription(WeedDetection, '/plevelai/detections', self.on_det, 10)
            self.pub_fire = self.create_publisher(Bool, '/plevelai/fire_cmd', 10)

        def on_aim(self, _vec: 'Vector2') -> None:  # type: ignore[name-defined]
            self.last_aim_time = time.time()

        def on_det(self, det: 'WeedDetection') -> None:  # type: ignore[name-defined]
            window = float(self.get_parameter('safety_window_ms').value) / 1000.0
            if time.time() - self.last_aim_time <= window and det.confidence >= 0.6:
                msg = Bool()
                msg.data = True
                self.pub_fire.publish(msg)

    rclpy.init()
    node = FireController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

