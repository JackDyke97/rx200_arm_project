#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint
from std_srvs.srv import SetBool


class RX200GripperControl(Node):
    def __init__(self):
        super().__init__('rx200_gripper_control')

        self._client = ActionClient(self, MoveGroup, '/move_action')
        while not self._client.wait_for_server(1.0):
            self.get_logger().info('Waiting for MoveIt Action Server...')

        self.group_name = 'interbotix_gripper'
        self.gripper_joint = 'left_finger'

        self.srv = self.create_service(SetBool, 'set_gripper', self.handle_set_gripper)
        self.get_logger().info('gripper ready.')

    def handle_set_gripper(self, request, response):
        success = self.move_gripper(open=request.data)
        response.success = success
        response.message = "gripper opened" if request.data else "gripper closed"
        return response

    def move_gripper(self, open=True):
        req = MotionPlanRequest()
        req.group_name = self.group_name
        req.start_state.is_diff = True

        jc = JointConstraint()
        jc.joint_name = self.gripper_joint
        jc.position = 0.035 if open else 0.0
        jc.tolerance_above = 0.01
        jc.tolerance_below = 0.01
        jc.weight = 1.0

        req.goal_constraints = [Constraints(joint_constraints=[jc])]

        goal = MoveGroup.Goal()
        goal.request = req

        # future = self._client.send_goal_async(goal)
        # rclpy.spin_until_future_complete(self, future)
        # handle = future.result()
        # if not handle.accepted:
        #     self.get_logger().error("gripper goal not able to be reached")
        #     return False

        # Send goal asynchronously — no spin_until_future_complete needed
        future = self._client.send_goal_async(goal)
        future.add_done_callback(lambda fut: self.handle_goal_response(fut, open))
        return True  # immediately return so service call completes

    def handle_goal_response(self, future, open_state):
        handle = future.result()
        if not handle or not handle.accepted:
            self.get_logger().error("rejected gripper target")
            return

        self.get_logger().info(f"gripper target accepted ({'open' if open_state else 'close'})")

        result_future = handle.get_result_async()
        result_future.add_done_callback(lambda fut: self.handle_result(fut, open_state))

    def handle_result(self, future, open_state):
        result = future.result()
        code = getattr(result.result.error_code, 'val', -1) if result else -1
        if code == 1:
            self.get_logger().info(f"gripper {'opened' if open_state else 'closed'} successfully")
        else:
            self.get_logger().warn(f"gripper movement failed with error code: {code}")

        


def main():
    rclpy.init()
    node = RX200GripperControl()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
