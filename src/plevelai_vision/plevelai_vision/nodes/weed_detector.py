import argparse
from typing import Optional

def main(argv: Optional[list[str]] = None) -> None:
    parser = argparse.ArgumentParser(description="Weed detector stub")
    parser.add_argument('--use_sim_time', action='store_true', help='Use simulated time')
    parser.add_argument('--detection_threshold', type=float, default=0.6)
    args = parser.parse_args(argv)

    try:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import Image
        from plevelai_interfaces.msg import WeedDetection
    except Exception:  # pragma: no cover
        print("[weed_detector] ROS 2 not available; stub exit.")
        return

    class WeedDetector(Node):
        def __init__(self) -> None:
            super().__init__('weed_detector')
            self.declare_parameter('use_sim_time', args.use_sim_time)
            self.declare_parameter('detection_threshold', args.detection_threshold)
            self.sub = self.create_subscription(Image, '/camera/image_raw', self.on_image, 10)
            self.pub = self.create_publisher(WeedDetection, '/plevelai/detections', 10)

        def on_image(self, _msg: Image) -> None:  # type: ignore[name-defined]
            det = WeedDetection()
            det.species = 'example_weed'
            det.x = 0.0
            det.y = 0.0
            det.confidence = float(self.get_parameter('detection_threshold').value)
            self.pub.publish(det)

    rclpy.init()
    node = WeedDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

