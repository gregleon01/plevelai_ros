import argparse
from typing import Optional

def main(argv: Optional[list[str]] = None) -> None:
    parser = argparse.ArgumentParser(description="Aim controller stub")
    parser.add_argument('--use_sim_time', action='store_true', help='Use simulated time')
    parser.add_argument('--aim_gain', type=float, default=1.0)
    args = parser.parse_args(argv)

    try:
        import rclpy
        from rclpy.node import Node
        from geometry_msgs.msg import Vector2
        from plevelai_interfaces.msg import WeedDetection
    except Exception:  # pragma: no cover
        print("[aim_controller] ROS 2 not available; stub exit.")
        return

    class AimController(Node):
        def __init__(self) -> None:
            super().__init__('aim_controller')
            self.declare_parameter('use_sim_time', args.use_sim_time)
            self.declare_parameter('aim_gain', args.aim_gain)
            self.pub = self.create_publisher(Vector2, '/plevelai/aim_cmd', 10)
            self.sub = self.create_subscription(WeedDetection, '/plevelai/detections', self.on_det, 10)

        def on_det(self, det: 'WeedDetection') -> None:  # type: ignore[name-defined]
            vec = Vector2()
            vec.x = det.x * float(self.get_parameter('aim_gain').value)
            vec.y = det.y * float(self.get_parameter('aim_gain').value)
            self.pub.publish(vec)

    rclpy.init()
    node = AimController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

