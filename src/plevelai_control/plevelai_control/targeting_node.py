import rclpy
from rclpy.node import Node
from plevelai_interfaces.msg import WeedDetection
from geometry_msgs.msg import Vector2

class Targeting(Node):
    def __init__(self):
        super().__init__('plevelai_targeting')
        self.declare_parameter('use_sim_time', True)
        self.sub = self.create_subscription(WeedDetection, '/plevelai/detections', self.cb, 10)
        self.pub = self.create_publisher(Vector2, '/plevelai/aim_cmd', 10)

    def cb(self, det: WeedDetection):
        v = Vector2(x=det.x, y=det.y)
        self.pub.publish(v)

def main():
    rclpy.init(); n = Targeting()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    n.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()
