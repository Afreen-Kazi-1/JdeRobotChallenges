import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
import numpy as np

class LaserScanVisualizer(Node):
    def __init__(self):
        super().__init__('laserscan_visualizer')
        self.subscription = self.create_subscription(LaserScan, '/scan', self.callback, 10)
        self.fig, self.ax = plt.subplots(subplot_kw={'projection': 'polar'})
        self.lidar_data = None
        self.angles = None

    def callback(self, msg):
        self.lidar_data = np.array(msg.ranges)
        self.angles = np.linspace(msg.angle_min, msg.angle_max, len(self.lidar_data))
        self.update_plot()

    def update_plot(self):
        self.ax.clear()
        self.ax.set_title('LaserScan Data')
        self.ax.set_ylim(0, max(self.lidar_data) + 1)
        self.ax.scatter(self.angles, self.lidar_data, c='r', s=10, label="LiDAR Points")
        self.ax.legend()
        plt.pause(0.1)

def main(args=None):
    rclpy.init(args=args)
    node = LaserScanVisualizer()
    plt.ion()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
