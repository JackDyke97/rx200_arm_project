#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener
import tf2_ros

class CentroidTFNode(Node):
    def __init__(self):
        super().__init__('centroid_tf_node')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.sub_centroid = self.create_subscription(
            PointStamped,
            '/detected_block',
            self.centroid_cb,
            10
        )

        self.pub_centroid_robot = self.create_publisher(
            PointStamped,
            '/detected_block_robot',
            10
        )

        self.get_logger().info('Centroid TF2 transform node running')

    def centroid_cb(self, msg):
        try:
            target_frame = 'base_link'
            transformed = self.tf_buffer.transform(
                msg,
                target_frame,
                timeout=rclpy.duration.Duration(seconds=0.5)
            )

            self.pub_centroid_robot.publish(transformed)

            self.get_logger().info(
                f'Transformed centroid to {target_frame}:'
                f'({transformed.point.x: .3f}),'
                f'{transformed.point.y: .3f}, '
                f'{transformed.point.z: .3f}'
            )

        except Exception as e:
            self.get_logger().warn(f'TF transform failed: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = CentroidTFNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()