import rclpy
import ros2_numpy as rnp
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import PointCloud2

class VoxelGridFilter(Node):
    def __init__ (self):
        super().__init__('voxel_grid_filter_node')

        #leaf size parameter in meters
        self.leaf_size = 0.01

        self.sub_leaf_size = self.create_subscription(Float32, '/voxelgrid/leaf_size', self.leaf_size_cb, 10)

        self.sub_pointcloud = self.create_subscription(PointCloud2, '/cropbox/filtered_points', self.pointcloud_cb, 10)

        self.pub_filtered = self.create_publisher(PointCloud2, '/voxelgrid/filtered_points', 10)

        self.get_logger().info('VoxelGrid filter started')

    def leaf_size_cb(self, msg):
            self.leaf_size = float(msg.data)
            self.get_logger().info(f'Leaf size is now: {self.leaf_size}')

    def pointcloud_cb(self, msg):

            points = rnp.point_cloud2.pointcloud2_to_array(msg)
            xyz = rnp.point_cloud2.get_xyz_points(points)

            if xyz.shape[0] == 0:
                return
            
            voxel_indices = np.floor(xyz / self.leaf_size).astype(np.int32)
            unique_indices = np.unique(voxel_indices, axis=0, return_index=True)
            filtered_xyz = xyz[unique_indices]

            filtered_msg = rnp.point_cloud2.array_to_pointcloud2(filtered_xyz,frame_id=msg.header.frame_id)

            self.pub_filtered.publish(filtered_msg)

def main(args=None):
      rclpy.init(args=args)
      node = VoxelGridFilter()
      rclpy.spin(node)
      node.destroy_node()
      rclpy.shutdown()

if __name__ == '__main__':
      main()