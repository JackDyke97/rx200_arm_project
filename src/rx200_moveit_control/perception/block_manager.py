#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool
import time
import math

class BlockManagerNode(Node):
    def __init__ (self):
        super().__init__('block_manager_node')

        self.sub_block = self.create_subscription(
            PointStamped,
            '/detected_block_robot',
            self.block_cb,
            10
        )

        self.pub_target = self.create_publisher(
            PointStamped,
            '/target_block',
            10
        )

        self.pub_flag = self.create_publisher(
            Bool,
            '/block_ready',
            10
        )

        self.blocks = {}

        self.merge_distance = 0.03 # 3 cm

        self.timeout = 2.0

        self.get_logger().info('Block manager is running')

    def block_cb(self, msg):
        px, py, pz = msg.point.x, msg.point.y, msg.point.z
        new_point = (px, py, pz)

        merged = False
        for block_id, data in self.blocks.items():
            old_point = data["point"]

            if self.dist(new_point, old_point) < self.merge_distance:
                self.blocks[block_id]['point'] = new_point
                self.blocks[block_id]['timestamp'] = time.time()
                merged = True
                break

        if not merged:
            new_id = len(self.blocks) + 1
            self.blocks[new_id] = {
                'point': new_point,
                'timestamp': time.time()
            }
            self.get_logger().info(f'new block detected: ID {new_id} at {new_point}')

        self.evaluate_blocks()

    def evaluate_blocks(self):
        now = time.time()

        stale = [bid for bid, data in self.blocks.items()
                 if now - data['timestamp'] > self.timeout]
        
        for bid in stale:
            del self.blocks[bid]
            self.get_logger().info(f'removed odl block id {bid}')

        if len(self.blocks) == 0:
            return
        
        selected_id, selected_block = self.select_block()

        ps = PointStamped()
        ps.header.frame_id = 'base_link'
        ps.point.x = float(selected_block['point'][0])
        ps.point.y = float(selected_block['point'][1])
        ps.point.z = float(selected_block['point'][2])

        self.pub_target.publish(ps)

        flag = Bool()
        flag.data = True
        self.pub_flag.publish(flag)

        self.get_logger().info(f'target block: ID {selected_id} at {selected_block['point']}')

    def select_block(self):
        def distance(p):
            return math.sqrt(p[0]**2 + p[1]**2 + p[2]**2)
        
        best_id = None
        best_dist = 999.0

        for block_id, data in self.blocks.items():
            d = distance(data['point'])
            if d < best_dist:
                best_dist = d
                best_id = block_id

        return best_id, self.blocks[best_id]
    
    def dist(a, b):
        return math.sqrt(
            (a[0] - b[0])**2 + (a[1] - b[1])**2 + (a[2] - b[2])**2
        )
    
def main (args=None):
    rclpy.init(args=args)
    node = BlockManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()