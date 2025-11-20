#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math

class CameraTF(Node):
    def __init__(self):
        super().__init__('camera_static_tf')

        #create the broadcaster 
        self.broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        t = TransformStamped()

        #parent and child frames
        t.header.frame_id = "base_link"
        t.child_frame_id = "camera_link"

        #values for camera mount (cm)**CHANGE THESE LATER YOU NERD**
        t.transform.translation.x = 0.30
        t.transform.translation.y = 0.00
        t.transform.translation.z = 0.50

        roll = 0.0
        pitch = -math.pi/4
        yaw = 0.0

        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        t.transform.rotation.w = cr * cp * cy + sr * sp * sy
        t.transform.rotation.x = sr * cp * cy - cr * sp * sy
        t.transform.rotation.y = cr * sp * cy + sr * cp * sy
        t.transform.rotation.z = cr * cp * sy - sr * sp * cy

        self.broadcaster.sendTransform(t)
        
        self.get_logger().info('published static transform camera link to base link')

def main(args=None):
    rclpy.init(args=args)
    node = CameraTF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
