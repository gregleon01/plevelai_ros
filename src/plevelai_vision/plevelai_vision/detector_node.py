import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from plevelai_interfaces.msg import WeedDetection

class Detector(Node):
    def __init__(self):
        super().__init__('plevelai_detector')
        self.declare_parameter('use_sim_time', True)
        self.sub = self.create_subscription(Image, '/camera/image_raw', self.cb, 10)
        self.pub = self.create_publisher(WeedDetection, '/plevelai/detections', 10)
        self.get_logger().info('Detector node ready (stub).')

    def cb(self, _msg: Image):
        det = WeedDetection(species='mock_weed', x=0.42, y=0.13, confidence=0.88)
        self.pub.publish(det)

def main():
    rclpy.init()
    n = Detector()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    n.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()
