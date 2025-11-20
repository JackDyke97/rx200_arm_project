import rclpy
import ros2_numpy as rnp
import numpy as np
import open3d as o3d
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import PointCloud2

class SegmentationFilter(Node):
    def __init__(self):
        super().__init__('segmentation_filter_node')

        self.dist_thresh = 0.01
        self.max_iter = 100

        self.sub_dist_thresh = self.create_subscription(Float32, '/seg/dist_thresh', self.dist_thresh_cb, 10)
        self.sub_iteration = self.create_subscription(Float32, '/seg/max_iter', self.max_iter_cb, 10)
        self.sub_cloud = self.create_subscription(PointCloud2, '/voxelgrid/filtered_points', self.pointcloud_cb, 10)
        
        self.pub_filtered = self.create_publisher(PointCloud2, '/segmentation/filtered_points',10)

        self.get_logger().info('Segmentation filter node running')

    def dist_thresh_cv(self, msg):
        self.dist_thresh = float(msg.data)
        self.get_logger().info(f'Updating threshold distance: {self.dist_thresh}')

    def max_iter_cb(self, msg):
        self.max_iter = int(msg.data)
        self.get_logger().info(f'Updating maximun iterations: {self.max_iter}')

    def pointcloud_cb(self, msg):

        points = rnp.point_cloud2.pointcloud2_to_array(msg)
        xyz = rnp.point_cloud2.get_xyz_points(points)

        if xyz.shape[0] == 0:
            return
        
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(xyz)

        plane_model, inliers = cloud.segment_plane(
            distance_threshold=self.dist_thresh,
            ransac_n=3,
            num_iterations=self.max_iter
        )

        cloud_filtered = cloud.select_by_index(inliers, invert=True)

        filtered_xyz = np.asarray(cloud_filtered.points)

        filtered_msg = rnp.point_cloud2.array_to_pointcloud2(filtered_xyz, frame_id=msg.header.frame_id)

        self.pub_filtered.publish(filtered_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SegmentationFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
