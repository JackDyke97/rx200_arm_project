import rclpy
import ros2_numpy as rnp
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import PointCloud2

class CropBoxFilter(Node):
    def __init__(self):
        super().__init__('crop_box_filter_node')

        self.x_min = 0.0
        self.x_max = 0.0
        self.y_min = 0.0
        self.y_max = 0.0
        self.z_min = 0.0
        self.z_max = 0.0


        self.sub_x_min = self.create_subscription(Float32, '/cropbox/x_min', self.callback_x_min, 10)
        self.sub_x_max = self.create_subscription(Float32, '/cropbox/x_max', self.callback_x_max, 10)
        self.sub_y_min = self.create_subscription(Float32, '/cropbox/y_min', self.callback_y_min, 10)
        self.sub_y_max = self.create_subscription(Float32, '/cropbox/y_max', self.callback_y_max, 10)
        self.sub_z_min = self.create_subscription(Float32, '/cropbox/z_min', self.callback_z_min, 10)
        self.sub_z_max = self.create_subscription(Float32, '/cropbox/z_max', self.callback_z_max, 10)

        self.sub_pointcloud = self.create_subscription(PointCloud2, '/camera/points', self.pointcloud_cb, 10)

        self.pub_filtered = self.create_publisher(PointCloud2, '/cropbox/filtered_points', 10)

        self.get_logger().info('The crop box filter node started')

    def callback_x_min(self,msg):
        self.x_min = msg.data
        self.get_logger().info(f'updating x min to {self.x_min}')
    
    def callback_x_max(self,msg):
        self.x_max = msg.data
        self.get_logger().info(f'updating x max to {self.x_max}')

    def callback_y_min(self,msg):
        self.y_min = msg.data
        self.get_logger().info(f'updating y min to {self.y_min}')

    def callback_y_max(self,msg):
        self.y_max = msg.data
        self.get_logger().info(f'updating y max to {self.y_max}')

    def callback_z_min(self,msg):
        self.z_min = msg.data
        self.get_logger().info(f'updating z min to {self.z_min}')

    def callback_z_max(self,msg):
        self.z_max = msg.data
        self.get_logger().info(f'updating z max to {self.z_max}')

    def pointcloud_cb(self, msg):
        
        points = rnp.point_cloud2.pointcloud2_to_array(msg)
        xyz = rnp.point_cloud2.get_xyz_points(points)

        mask = (
            (xyz[:, 0] >= self.x_min) & (xyz[:, 0] <= self.x_max) &
            (xyz[:, 1] >= self.y_min) & (xyz[:, 1] <= self.y_max) &
            (xyz[:, 2] >= self.z_min) & (xyz[:, 2] <= self.z_max) 
        )

        filtered_xyz = xyz[mask]

        filtered_msg = rnp.point_cloud2.array_to_pointcloud2(filtered_xyz, frame_id=msg.header.frame_id)

        self.pub_filtered.publish(filtered_msg)
       

def main(args=None):
    rclpy.init(args=args)
    node = CropBoxFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()