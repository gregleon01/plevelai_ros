import argparse
from typing import Optional

def main(argv: Optional[list[str]] = None) -> None:
    parser = argparse.ArgumentParser(description="Mock laser actuator")
    parser.add_argument('--use_sim_time', action='store_true', help='Use simulated time')
    parser.add_argument('--safety_window_ms', type=int, default=500)
    args = parser.parse_args(argv)

    try:
        import rclpy
        from rclpy.node import Node
        from geometry_msgs.msg import Vector2
        from std_msgs.msg import Bool
    except Exception:  # pragma: no cover
        print("[mock_laser] ROS 2 not available; stub exit.")
        return

    class MockLaser(Node):
        def __init__(self) -> None:
            super().__init__('mock_laser')
            self.declare_parameter('use_sim_time', args.use_sim_time)
            self.declare_parameter('safety_window_ms', args.safety_window_ms)
            self.sub_aim = self.create_subscription(Vector2, '/plevelai/aim_cmd', self.on_aim, 10)
            self.sub_fire = self.create_subscription(Bool, '/plevelai/fire_cmd', self.on_fire, 10)
            self.last_aim = (0.0, 0.0)

        def on_aim(self, v: 'Vector2') -> None:  # type: ignore[name-defined]
            self.last_aim = (v.x, v.y)
            self.get_logger().info(f"[mock_laser] Aim to x={v.x:.3f}, y={v.y:.3f}")

        def on_fire(self, b: 'Bool') -> None:  # type: ignore[name-defined]
            if b.data:
                self.get_logger().info(f"[mock_laser] FIRE at aim={self.last_aim}")
            else:
                self.get_logger().info("[mock_laser] Cease fire")

    rclpy.init()
    node = MockLaser()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

