import argparse
import time
from typing import Optional

def main(argv: Optional[list[str]] = None) -> None:
    parser = argparse.ArgumentParser(description="Mock camera publisher")
    parser.add_argument('--use_sim_time', action='store_true', help='Use simulated time')
    parser.add_argument('--image_rate_hz', type=float, default=5.0)
    parser.add_argument('--image_folder', type=str, default='samples/images')
    args = parser.parse_args(argv)

    try:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import Image
    except Exception:  # pragma: no cover - mac scaffolding without ROS 2
        print("[mock_camera] ROS 2 not available; stub exit.")
        return

    class MockCamera(Node):
        def __init__(self) -> None:
            super().__init__('mock_camera')
            self.declare_parameter('use_sim_time', args.use_sim_time)
            self.declare_parameter('image_rate_hz', args.image_rate_hz)
            self.declare_parameter('image_folder', args.image_folder)
            self.pub = self.create_publisher(Image, '/camera/image_raw', 10)
            period = 1.0 / max(0.1, float(self.get_parameter('image_rate_hz').value))
            self.timer = self.create_timer(period, self._tick)
            self.seq = 0

        def _tick(self) -> None:
            msg = Image()
            msg.height = 1
            msg.width = 1
            msg.encoding = 'rgb8'
            msg.step = 3
            msg.data = bytes([0, 0, 0])
            self.pub.publish(msg)
            self.seq += 1

    rclpy.init()
    node = MockCamera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

