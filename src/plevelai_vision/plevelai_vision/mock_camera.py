import os, time, glob
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class MockCamera(Node):
    def __init__(self):
        super().__init__('mock_camera')
        self.declare_parameter('use_sim_time', True)
        self.pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.folder = os.path.join(os.path.dirname(__file__), '..', 'sample_images')
        self.paths = sorted(glob.glob(os.path.join(self.folder, '*')))
        self.idx = 0
        self.timer = self.create_timer(0.2, self.tick)  # 5 Hz

    def tick(self):
        msg = Image()  # empty stub; just triggers downstream
        self.pub.publish(msg)
        self.idx = (self.idx + 1) % max(1, len(self.paths))

def main():
    rclpy.init(); n = MockCamera()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    n.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()
