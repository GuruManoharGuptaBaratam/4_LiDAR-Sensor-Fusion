import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PointStamped
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np


class MeasurementNode(Node):
    def __init__(self):
        super().__init__('measurement_node')

        self.sub = self.create_subscription(
            PointCloud2,
            '/points_fused',
            self.cloud_callback,
            10
        )

        self.pub = self.create_publisher(
            PointStamped,
            '/object_measurement',
            10
        )

    def cloud_callback(self, msg):
        points = np.array([
            [p[0], p[1]] for p in pc2.read_points(msg, skip_nans=True)
        ])

        if len(points) == 0:
            return

        centroid = points.mean(axis=0)

        out = PointStamped()
        out.header = msg.header
        out.point.x = float(centroid[0])
        out.point.y = float(centroid[1])
        out.point.z = 0.0

        self.pub.publish(out)


def main():
    rclpy.init()
    node = MeasurementNode()
    rclpy.spin(node)
    rclpy.shutdown()
