import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class MockLaser(Node):
    def __init__(self):
        super().__init__('mock_laser')
        self.declare_parameter('use_sim_time', True)
        self.sub = self.create_subscription(Bool, '/plevelai/fire_cmd', self.cb, 10)

    def cb(self, msg: Bool):
        if msg.data:
            self.get_logger().info('FIRE (200 ms)')
            time.sleep(0.2)

def main():
    rclpy.init(); n = MockLaser()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    n.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()
