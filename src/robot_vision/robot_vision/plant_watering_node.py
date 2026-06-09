import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    WorkspaceParameters,
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
)
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header
from vision_msgs.msg import Detection3DArray
import numpy as np


class PlantWateringNode(Node):

    def __init__(self):
        super().__init__('plant_watering_node')

        self._action_client = ActionClient(
            self,
            MoveGroup,
            '/move_group'
        )

        self.plant_sub = self.create_subscription(
            Detection3DArray,
            '/plant_detections',
            self.plant_detection_callback,
            10
        )

        self.busy = False
        self.get_logger().info('Plant watering node ready')

    def plant_detection_callback(self, msg):
        if self.busy:
            return
        if len(msg.detections) == 0:
            return

        # Take the first detected plant
        detection = msg.detections[0]
        x = detection.bbox.center.position.x
        y = detection.bbox.center.position.y
        z = detection.bbox.center.position.z

        self.get_logger().info(f'Plant detected at x={x:.3f} y={y:.3f} z={z:.3f}')
        self.busy = True
        self.move_arm_to_plant(x, y, z)

    def move_arm_to_plant(self, x, y, z):
        self.get_logger().info('Waiting for MoveIt action server...')
        self._action_client.wait_for_server()

        # Build the target pose
        target_pose = PoseStamped()
        target_pose.header.frame_id = 'base_link'
        target_pose.header.stamp = self.get_clock().now().to_msg()
        target_pose.pose.position.x = x
        target_pose.pose.position.y = y
        target_pose.pose.position.z = z

        # Keep end effector pointing forward (no rotation constraint)
        target_pose.pose.orientation.x = 0.0
        target_pose.pose.orientation.y = 0.0
        target_pose.pose.orientation.z = 0.0
        target_pose.pose.orientation.w = 1.0

        # Position constraint — allow 5cm tolerance around target
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = 'base_link'
        position_constraint.link_name = 'camera_link'
        position_constraint.target_point_offset.x = 0.0
        position_constraint.target_point_offset.y = 0.0
        position_constraint.target_point_offset.z = 0.0

        bounding_volume = BoundingVolume()
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [0.05]  # 5cm radius tolerance
        bounding_volume.primitives.append(primitive)
        bounding_volume.primitive_poses.append(target_pose.pose)
        position_constraint.constraint_region = bounding_volume
        position_constraint.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(position_constraint)

        # Build the motion plan request
        request = MotionPlanRequest()
        request.group_name = 'arm'
        request.goal_constraints.append(constraints)
        request.num_planning_attempts = 10
        request.allowed_planning_time = 5.0
        request.max_velocity_scaling_factor = 0.3
        request.max_acceleration_scaling_factor = 0.3

        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3

        self.get_logger().info('Sending arm to plant position...')
        send_goal_future = self._action_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('MoveIt rejected the goal')
            self.busy = False
            return
        self.get_logger().info('MoveIt accepted goal — arm moving...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        pass  # can log feedback here if needed

    def result_callback(self, future):
        result = future.result().result
        error_code = result.error_code.val

        if error_code == 1:
            self.get_logger().info('Arm reached the plant — watering now!')
            self.trigger_watering()
        else:
            self.get_logger().error(f'MoveIt failed with error code: {error_code}')

        # Return arm to home after watering
        self.get_logger().info('Returning arm to home position...')
        self.move_to_home()
        self.busy = False

    def trigger_watering(self):
        # TODO: publish to your water pump topic here
        # e.g. self.pump_pub.publish(...)
        self.get_logger().info('Watering triggered for 3 seconds')

    def move_to_home(self):
        # Send arm back to home named pose
        request = MotionPlanRequest()
        request.group_name = 'arm'
        request.num_planning_attempts = 5
        request.allowed_planning_time = 3.0
        request.max_velocity_scaling_factor = 0.5
        request.max_acceleration_scaling_factor = 0.5

        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options.plan_only = False

        self._action_client.send_goal_async(goal)


def main(args=None):
    rclpy.init(args=args)
    node = PlantWateringNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()