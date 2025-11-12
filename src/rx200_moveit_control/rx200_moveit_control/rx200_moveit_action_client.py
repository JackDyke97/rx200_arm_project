#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    JointConstraint,
)
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
import math, ast
from std_srvs.srv import SetBool


class MoveItEEClient(Node):
    def __init__(self):
        super().__init__("rx200_moveit_control")

        # set parameters for start, pick and place positions
        self.declare_parameter("start", [0.25, 0.0, 0.30])
        self.declare_parameter("pick", [0.25, 0.0, 0.18])
        self.declare_parameter("place", [-0.20, 0.0, 0.18])

        # get parameters from launch file
        def parse_param(name):
            val = self.get_parameter(name).value
            return ast.literal_eval(val) if isinstance(val, str) else val

        self.start = parse_param("start")
        self.pick = parse_param("pick")
        self.place = parse_param("place")
        self.get_logger().info(f"Start: {self.start}, Pick: {self.pick}, Place: {self.place}")

        self._arm_client = ActionClient(self, MoveGroup, "/move_action")
        while not self._arm_client.wait_for_server(1.0):
            self.get_logger().info("Waiting for Action Server")

        self._gripper_client = self.create_client(SetBool, 'set_gripper')
        while not self._gripper_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info("waiting for gripper service")

        self.group_arm = "interbotix_arm"
        self.group_gripper = "interbotix_gripper"
        self.ee_link = "rx200/ee_gripper_link"
        self.base_link = "rx200/base_link"
        self.gripper_joint = "left_finger"

        self.get_logger().info("Arm and gripper nodes ready")

    #find the smallest angle
    def _shortest_yaw(self, a, b):
        return (b - a + math.pi) % (2 * math.pi) - math.pi

    #checks if robot can reach positon
    def _reachable(self, x, y, z):
        r = math.hypot(x, y)
        #return 0.12 <= z <= 0.40 and 0.08 <= r <= 0.28
        return 0.12 <= z <= 0.45 and 0.08 <= r <= 0.43
    
    #raises height when near base to prevent colliding 
    def _safe_height(self, z, x, yaw=0.0):
        if abs(x) < 0.08 and z < 0.40:
            self.get_logger().info("Avoiding the base")
            z = 0.32

        if x < -0.05 :
            yaw = math.pi
        elif x > 0.05:
            yaw = 0.0
            
        return z, yaw
    
    #adds a point above the base to prevent colliding    
    def _safe_mid(self, a, b, z=0.28, y_eps=0.05):
        xm = 0.5 * (a[0] + b[0])
        ym = y_eps if abs(a[1]) < 1e-3 and abs(b[1]) < 1e-3 else 0.0
        return [xm, ym, z]
    
    #moves to a safe position to prevent singularity
    def _prebend(self):
        self.get_logger().info("Moving to safe pose")
        self.send_pose(0.18, 0.05, 0.32, yaw=0.0)


    def send_pose(self, x, y, z, yaw=0.0):
        #z, yaw = self._safe_height(z, x, yaw)

        if not self._reachable(x, y, z):
            self.get_logger().warn(f"Pose out of reach, skipping {x, y, z}")
            return False

        pose = PoseStamped()
        pose.header.frame_id = self.base_link
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)

        # convert yaw to quaternion
        q = quaternion_from_euler(0.0, 0.0, yaw)
        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = q

        req = MotionPlanRequest()
        req.group_name = self.group_arm
        req.allowed_planning_time = 15.0
        req.num_planning_attempts = 5
        req.start_state.is_diff = True
        req.max_velocity_scaling_factor = 0.4
        req.max_acceleration_scaling_factor = 0.4

        box = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.07, 0.07, 0.07])
        pos = PositionConstraint()
        pos.header.frame_id = self.base_link
        pos.link_name = self.ee_link
        pos.constraint_region.primitives = [box]
        pos.constraint_region.primitive_poses = [pose.pose]

        ori = OrientationConstraint()
        ori.header.frame_id = self.base_link
        ori.link_name = self.ee_link
        ori.orientation = pose.pose.orientation
        ori.absolute_x_axis_tolerance = 0.6
        ori.absolute_y_axis_tolerance = 0.6
        ori.absolute_z_axis_tolerance = 0.6

        req.goal_constraints = [Constraints(position_constraints=[pos], orientation_constraints=[ori])]
        goal = MoveGroup.Goal()
        goal.request = req

        future = self._arm_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if not handle or not handle.accepted:
            self.get_logger().error("goal rejected")
            return False

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=30.0)
        result_handle = result_future.result()
        code = getattr(result_handle.result.error_code, "val", -1) if result_handle else -1
        self.get_logger().info(f"[RESULT] error: {code}")
        return code == 1

    def control_gripper(self, open_state: bool):
        req = SetBool.Request()
        req.data = open_state
        future = self._gripper_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        if result and result.success:
            self.get_logger().info(result.message)
        else:
            self.get_logger().error("Failed to control gripper")

    #moves in straight line between waypoints
    # def cartesian_path(self, start, end, steps=5, yaw_start=0.0, yaw_end=None):
    #     if len(end) == 4:
    #         x1, y1, z1, end_yaw = end
    #         if yaw_end is None:
    #             yaw_end = end_yaw
    #     else:
    #         x1, y1, z1 = end
    #         if yaw_end is None:
    #             yaw_end = 0.0

    #     if len(start) == 4:
    #         x0, y0, z0, yaw_start = start
    #     else:
    #         x0, y0, z0 = start

    #     dyaw = self._shortest_yaw(yaw_start, yaw_end)

    #     for i in range(steps + 1):
    #         t = i / steps
    #         xi = x0 + (x1 - x0) * t
    #         yi = y0 + (y1 - y0) * t
    #         zi = z0 + (z1 - z0) * t
    #         yaw = yaw_start + dyaw * t

    #         zi, yaw = self._safe_height(zi, xi, yaw)

    #         self.get_logger().info(f"Step {i}/{steps}: ({xi:.3f}, {yi:.3f}, {zi:.3f}), yaw={yaw:.2f}")

    #         if not self.send_pose(xi, yi, zi, yaw):
    #             self.get_logger().warn(f"Step {i} failed, trying again")
    #             if not self.send_pose(xi, yi, zi, yaw):
    #                 self.get_logger().warn(f"Step {i} failed again, skipping")
    #                 return False

    #     return True

    def move_sequence(self):
        self._prebend()
        self.get_logger().info("Starting pick and place")

        # move to pick position
        self.control_gripper(True)   # open gripper
        self.send_pose(*self.pick)
        # self.cartesian_path(self.start, self.pick, steps=5)
        self.control_gripper(False)  # close gripper

        # lift object up
        lifted_pick = [self.pick[0], self.pick[1], self.pick[2] + 0.12]
        self.send_pose(*lifted_pick)
        # self.cartesian_path(self.pick, lifted_pick, steps=3)

        # move across
        mid_pose = self._safe_mid(self.pick, self.place, z=0.30, y_eps=0.07)
        yaw_place = math.pi if self.place[0] < 0 else 0.0
        self.send_pose(*mid_pose)
        # self.cartesian_path(lifted_pick, mid_pose, steps=4, yaw_start=0.0, yaw_end=yaw_place)
        # self.cartesian_path(mid_pose, [self.place[0], self.place[1], 0.25], steps=4, yaw_start=yaw_place, yaw_end=yaw_place)

        # rotate if needed
        high_place = [self.place[0], self.place[1], 0.25]
        yaw_place = math.pi if self.place[0] < 0 else 0.0
        self.send_pose(*high_place, yaw=yaw_place)
        self.send_pose(*self.place)
        # self.cartesian_path(high_place, self.place, steps=3, yaw_start=0.0, yaw_end=yaw_place)

        # place object
        self.control_gripper(True)   # open gripper (release)

        # return to start
        lift_from_place = [self.place[0], self.place[1], 0.25]
        self.send_pose(*lift_from_place, yaw=yaw_place)
        self.send_pose(*self.start)
        # self.cartesian_path(self.place, lift_from_place, steps=3)
        # self.cartesian_path(lift_from_place, [self.start[0], self.start[1], 0.25], steps=4, yaw_start=yaw_place, yaw_end=0.0)
        # self.cartesian_path([self.start[0], self.start[1], 0.25], self.start, steps=2)

        self.get_logger().info("movement complete")




def main():
    rclpy.init()
    node = MoveItEEClient()
    node.move_sequence()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
