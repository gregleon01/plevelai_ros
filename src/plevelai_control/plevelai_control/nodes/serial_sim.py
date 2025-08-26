import argparse
from typing import Optional

def main(argv: Optional[list[str]] = None) -> None:
    parser = argparse.ArgumentParser(description="Serial simulator for Teensy comms")
    parser.add_argument('--use_sim_time', action='store_true', help='Use simulated time')
    parser.add_argument('--tick_ms', type=int, default=100)
    args = parser.parse_args(argv)

    try:
        import rclpy
        from rclpy.node import Node
    except Exception:  # pragma: no cover
        print("[serial_sim] ROS 2 not available; stub exit.")
        return

    class SerialSim(Node):
        def __init__(self) -> None:
            super().__init__('serial_sim')
            self.declare_parameter('use_sim_time', args.use_sim_time)
            self.declare_parameter('tick_ms', args.tick_ms)
            period = float(self.get_parameter('tick_ms').value) / 1000.0
            self.state = 'READY'
            self.timer = self.create_timer(period, self._tick)

        def _tick(self) -> None:
            order = ['READY', 'ARMED', 'FIRED', 'COOLING']
            self.state = order[(order.index(self.state) + 1) % len(order)]
            self.get_logger().info(f"[serial_sim] state={self.state}")

    rclpy.init()
    node = SerialSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

