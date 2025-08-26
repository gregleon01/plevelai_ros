import rclpy
from rclpy.node import Node
class SerialSim(Node):
    def __init__(self):
        super().__init__('serial_sim')
        self.declare_parameter('use_sim_time', True)
        self.get_logger().info('Serial simulator ready (no real I/O).')
def main():
    rclpy.init(); n = SerialSim()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    n.destroy_node(); rclpy.shutdown()
if __name__ == '__main__': main()
