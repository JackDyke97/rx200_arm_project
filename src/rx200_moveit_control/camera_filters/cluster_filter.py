import rclpy
import ros2_numpy as rnp
import numpy as np
import open3d as o3d
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import PointCloud2

class ClusterFilter(Node):
    def __init__(self):
        super().__init__('cluster_filter_node')

        self.cluster_min = 25
        self.cluster_max = 1000
        self.tolerance = 0.02

        self.sub_cluster_min = self.create_subscription(
            Float32,
            '/cluster/min_size',
            self.cluster_min_cb,
            10
        )

        self.sub_cluster_max = self.create_subscription(
            Float32,
            '/cluster/max_size',
            self.cluster_max_cb,
            10
        )

        self.sub_tolerance = self.create_subscription(
            Float32,
            '/cluster/tolerance',
            self.tolerance_cb,
            10
        )

        self.sub_cloud = self.create_subscription(
            PointCloud2,
            'ror/filtered_points',
            self.pointcloud_cb,
            10
        )

        self.pub_clustered = self.create_publisher(
            PointCloud2,
            '/cluster/clustered_points',
            10
        )

        self.get_logger().info('Cluster filter node started')

    def cluster_min_cb(self, msg):
        self.cluster_min = int(msg.data)
        self.get_logger().info(f'Updating cluster min: {self.cluster_min}')

    def cluster_max_cb(self, msg):
        self.cluster_max = int(msg.data)
        self.get_logger().info(f'Updating cluster max: {self.cluster_max}')

    def tolerance_cb(self, msg):
        self.tolerance = float(msg.data)
        self.get_logger().info(f'updating tolerance: {self.tolerance}')

    def pointcloud_cb(self, msg):

        points = rnp.point_cloud2.pointcloud2_to_array(msg)
        xyz = rnp.point_cloud2.get_xyz_points(points)

        if xyz.shape[0] == 0:
            return
        
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(xyz)

        labels = np.array(
            cloud.cluster_dbscan(
                eps=self.tolerance,
                min_points=self.cluster_min,
                print_progress=False
            )
        )

        max_label = labels.max()
        if max_label < 0:
            return
        
        colours = np.zeros((len(labels), 3))

        for cluster_id in range(max_label + 1):
            indices = np.where(labels == cluster_id)[0]
            if len(indices) < self.cluster_min or len(indices) > self.cluster_max:
                continue

            colour = np.random.rand(3)
            colours[indices] = colour
        
        cloud.colours = o3d.utility.Vector3dVector(colours)

        clustered_msg = rnp.point_cloud2.array_to_pointcloud2(
            np.c_[xyz, colours],
            frame_id=msg.header.frame_id
        )

        self.pub_clustered.publish(clustered_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ClusterFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
