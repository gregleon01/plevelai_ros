import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector2
from std_msgs.msg import Bool

class Firing(Node):
    def __init__(self):
        super().__init__('plevelai_firing')
        self.declare_parameter('use_sim_time', True)
        self.sub = self.create_subscription(Vector2, '/plevelai/aim_cmd', self.cb, 10)
        self.pub = self.create_publisher(Bool, '/plevelai/fire_cmd', 10)
        self.cooldown_s = 0.2
        self._last = 0.0

    def cb(self, _aim: Vector2):
        now = time.time()
        if now - self._last >= self.cooldown_s:
            self.pub.publish(Bool(data=True))
            self._last = now

def main():
    rclpy.init(); n = Firing()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    n.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()
