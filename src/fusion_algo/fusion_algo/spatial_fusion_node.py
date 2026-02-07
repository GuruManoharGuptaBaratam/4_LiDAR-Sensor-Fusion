#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from scipy.spatial.transform import Rotation as R

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import message_filters

class QuadLidarFusionNode(Node):
    def __init__(self):
        super().__init__('quad_lidar_fusion_node')


        self.declare_parameter('target_frame', 'base_link')
        self.target_frame = self.get_parameter('target_frame').value
        

        self.topics = [
            '/lidar_A/scan/points',
            '/lidar_B/scan/points',
            '/lidar_C/scan/points',
            '/lidar_D/scan/points'
        ]

      
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)


        self.subs = []
        for topic in self.topics:
            self.get_logger().info(f'Subscribing to: {topic}')
            self.subs.append(message_filters.Subscriber(self, PointCloud2, topic))


        self.sync = message_filters.ApproximateTimeSynchronizer(
            self.subs,
            queue_size=10,
            slop=0.1
        )
        self.sync.registerCallback(self.fusion_callback)


        self.pub_fused = self.create_publisher(PointCloud2, '/points_fused', 10)
        self.get_logger().info('Quad LiDAR Fusion Node is ready.')

    def fusion_callback(self, msg_a, msg_b, msg_c, msg_d):
        """Processes 4 incoming clouds simultaneously."""
        clouds = [msg_a, msg_b, msg_c, msg_d]
        transformed_points_list = []

        try:
            for cloud in clouds:

                trans = self.tf_buffer.lookup_transform(
                    self.target_frame, 
                    cloud.header.frame_id, 
                    rclpy.time.Time()
                )


                pts = self.cloud_to_numpy(cloud)


                if pts.size > 0:
                    pts_fused = self.apply_transform(pts, trans)
                    transformed_points_list.append(pts_fused)


            if not transformed_points_list:
                return


            fused_pts = np.vstack(transformed_points_list)

            fused_cloud = point_cloud2.create_cloud_xyz32(msg_a.header, fused_pts)
            fused_cloud.header.frame_id = self.target_frame
            
            self.pub_fused.publish(fused_cloud)

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f'TF Error: {e}')

    def cloud_to_numpy(self, cloud):
        """Converts PointCloud2 to Nx3 float32 array safely."""
        gen = point_cloud2.read_points(cloud, field_names=("x", "y", "z"), skip_nans=True)
        pts_list = list(gen)
        if not pts_list:
            return np.empty((0, 3), dtype=np.float32)
        

        return np.array(pts_list).view(np.float32).reshape(-1, 3)

    def apply_transform(self, points, transform_stamped):
        """Applies TF rotation and translation to the points."""
        q = transform_stamped.transform.rotation
        t = transform_stamped.transform.translation
        
        rotation = R.from_quat([q.x, q.y, q.z, q.w])
        translation = np.array([t.x, t.y, t.z])
        
        return rotation.apply(points) + translation

def main():
    rclpy.init()
    node = QuadLidarFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
