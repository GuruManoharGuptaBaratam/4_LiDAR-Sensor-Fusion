import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
import numpy as np
import time

class KalmanFilter:
    def __init__(self):
        self.X = np.zeros((4, 1))
        self.P = np.eye(4) * 10.0

        self.last_time = None

        self.Q = np.eye(4) * 0.1
        self.R = np.eye(2) * 0.5
        self.H = np.array([[1, 0, 0, 0],
                           [0, 1, 0, 0]])
        self.I = np.eye(4)

    def predict(self, dt):
        F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1,  0],
            [0, 0, 0,  1]
        ])

        self.X = F @ self.X
        self.P = F @ self.P @ F.T + self.Q

    def update(self, Z):
        Y = Z - self.H @ self.X
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)

        self.X = self.X + K @ Y
        self.P = (self.I - K @ self.H) @ self.P
class KalmanTrackerNode(Node):
    def __init__(self):
        super().__init__('kalman_tracker')

        self.kf = KalmanFilter()

        self.sub = self.create_subscription(
            PointStamped,
            '/object_measurement',
            self.measurement_callback,
            10
        )

        self.pub = self.create_publisher(
            Odometry,
            '/tracked_state',
            10
        )

    def measurement_callback(self, msg):
        now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self.kf.last_time is None:
            self.kf.last_time = now
            self.kf.X[0, 0] = msg.point.x
            self.kf.X[1, 0] = msg.point.y
            return

        dt = now - self.kf.last_time
        self.kf.last_time = now

        Z = np.array([[msg.point.x],
                      [msg.point.y]])

        self.kf.predict(dt)
        self.kf.update(Z)

        self.publish_state(msg.header)

    def publish_state(self, header):
        odom = Odometry()
        odom.header = header

        odom.pose.pose.position.x = float(self.kf.X[0])
        odom.pose.pose.position.y = float(self.kf.X[1])
        odom.twist.twist.linear.x = float(self.kf.X[2])
        odom.twist.twist.linear.y = float(self.kf.X[3])

        self.pub.publish(odom)
def main():
    rclpy.init()
    node = KalmanTrackerNode()
    rclpy.spin(node)
    rclpy.shutdown()
