import rclpy
import ros2_numpy as rnp
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PointStamped

class CentroidNode(Node):
    def __init__(self):
        super().__init__('centroid_node')

        self.sub_cloud = self.create_subscription(
            PointCloud2,
            '/cluster/clustered_points',
            self.pointcloud_cb,
            10
        )

        self.pub_blocks = self.create_publisher(
            PointStamped,
            '/detected_block',
            10
        )

        self.pub_blocks_list = self.create_publisher(
            PointCloud2,
            '/detected_blocks_cloud',
            10
        )

        self.get_logger().info('Centroid node running')

    def extract_rgb(self, array):
        rgb_uint32 = array['rgb'].view(np.uint32).reshape(-1)
        r = (rgb_uint32 >> 16) & 255
        g = (rgb_uint32 >> 8) & 255
        b = rgb_uint32 & 255
        return list(zip(r, g, b))

    def pointcloud_cb(self, msg):
        array = rnp.point_cloud2.pointcloud2_to_array(msg)
        xyz = rnp.point_cloud2.get_xyz_points(array)

        if 'rgb' not in array.dtype.names:
            self.get_logger().warn('no rgb field, clustering not available')
            return
        
        rgb_tuples = self.extract_rgb(array)
        unique_colours = list(set(rgb_tuples))

        centroids = []

        for colour in unique_colours:
            indices = [i for i, c in enumerate(rgb_tuples) if c == colour]
            if len(indices) == 0:
                continue

            cluster_points = xyz[indices]
            centroid = cluster_points.mean(axis=0)
            centroids.append(centroid)

            ps = PointStamped()
            ps.header.frame_id = msg.header.frame_id
            ps.header.stamp = msg.header.stamp
            ps.point.x = float(centroid[0])
            ps.point.y = float(centroid[1])
            ps.point.z = float(centroid[2])

            self.pub_blocks.publish(ps)
            

        if len(centroids) > 0:
            centroid_cloud = np.array(centroids, dtype=np.float32)
            cloud_msg = rnp.point_cloud2.array_to_pointcloud2(
                centroid_cloud,
                frame_id=msg.header.frame_id
            )
            self.pub_blocks_list.publish(cloud_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CentroidNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
