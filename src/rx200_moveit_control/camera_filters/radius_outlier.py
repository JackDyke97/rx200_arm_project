import rclpy
import ros2_numpy as rnp
import numpy as np
import open3d as o3d
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import PointCloud2

class RadiusOutlier(Node):
    def __init__(self):
        super().__init('radius_outlier_filter_node')

        self.min_neighbours = 2
        self.radius = 0.01

        self.sub_min_neighbours = self.create_subscription(
            Float32,
            '/ror/min_neighbors',
            self.min_neighbours_cb,
            10
        )

        self.sub_radius = self.create_subscription(
            Float32,
            '/ror/radius',
            self.radius_cb,
            10
        )

        self.sub_cloud = self.create_subscription(
            PointCloud2,
            '/segmentation/filtered_points',
            self.pointcloud_cb,
            10
        )

        self.pub_filtered = self.create_publisher(PointCloud2, '/ror/filtered_points', 10)

        self.get_logger().info("Radisu outlier node started")

    def min_neighbours_cb(self, msg):
        self.min_neighbours = int(msg.data)
        self.get_logger().info(f"updating min neighbours: {self.min_neighbours}")

    def radius_cb(self, msg):
        self.radius = float(msg.data)
        self.get_logger().info(f'Updatin the radius: {self.radius}')

    def pointcloud_cb(self,msg):

        points = rnp.point_cloud2.pointcloud2_to_array(msg)
        xyz = rnp.point_cloud2.get_xyz_points(points)

        if xyz.shape[0] == 0:
            return
        
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(xyz)

        cloud_filtered, ind = cloud.remove_radius_outlier(
            nb_points=self.min_neighbours,
            radius=self.radius
        )

        filtered_xyz = np.asarray(cloud_filtered.points)


        filtered_msg = rnp.point_cloud2.array_to_pointcloud2(
            filtered_xyz,
            frame_id=msg.header.frame_id
        )

        self.pub_filtered.publish(filtered_msg)

def main(args=None):
        rclpy.init(args=args)
        node = RadiusOutlier()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()