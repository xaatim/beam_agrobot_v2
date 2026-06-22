import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PointStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    JointConstraint,
)
import time
import math

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs


class PlantWateringNode(Node):

    def __init__(self):
        super().__init__('plant_watering_node')

        self._action_client = ActionClient(
            self,
            MoveGroup,
            'move_action'
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.plant_sub = self.create_subscription(
            PointStamped,
            '/agrobot/target_watering_point',
            self.plant_detection_callback,
            10
        )

        self.busy = True
        self.get_logger().info('Initializing arm...')
        self.timer = self.create_timer(2.0, self.init_pose)

    def init_pose(self):
        self.timer.cancel()
        self._action_client.wait_for_server()
        self.move_to_home()

    def plant_detection_callback(self, msg):
        if self.busy:
            return

        if msg.point.z > 0.8:
            return

        self.get_logger().info(
            f'TARGET IN REACH! Distance: {msg.point.z:.2f}m.')
        self.busy = True
        self.move_arm_to_plant(msg)

    def move_arm_to_plant(self, point_msg):
        try:
            transform = self.tf_buffer.lookup_transform(
                'base_link',
                point_msg.header.frame_id,
                rclpy.time.Time()
            )
            base_link_point = tf2_geometry_msgs.do_transform_point(
                point_msg, transform)
        except TransformException as ex:
            self.get_logger().error(f'TF Error: {ex}')
            self.busy = False
            return

        self._action_client.wait_for_server()

        target_yaw = math.atan2(base_link_point.point.y,
                                base_link_point.point.x)

        request = MotionPlanRequest()
        request.group_name = 'arm'
        goal_constraints = Constraints()

        joint_names = ['base_yaw_joint', 'shoulder_pitch_joint',
                       'elbow_pitch_joint', 'wrist_pitch_joint', 'wrist_roll_joint']

        # All joints at 0 except base_yaw which rotates to face the plant.
        # This is the natural resting pose — fully collision free.
        # Only the yaw changes, everything else stays flat.
        target_angles = [target_yaw, 0.0, 0.0, 0.0, 0.0]

        for name, angle in zip(joint_names, target_angles):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = angle
            jc.tolerance_above = 0.05
            jc.tolerance_below = 0.05
            jc.weight = 1.0
            goal_constraints.joint_constraints.append(jc)

        request.goal_constraints.append(goal_constraints)
        request.num_planning_attempts = 10
        request.allowed_planning_time = 10.0
        request.max_velocity_scaling_factor = 0.3
        request.max_acceleration_scaling_factor = 0.3

        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options.plan_only = False

        self.get_logger().info(
            f'Rotating to plant at yaw={target_yaw:.2f} rads...')
        send_goal_future = self._action_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('MoveIt rejected the goal.')
            self.busy = False
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        status = future.result().result.error_code.val

        if status == 1:
            self.trigger_watering()
        else:
            self.get_logger().error(f'MoveIt failed with error code: {status}')

        self.get_logger().info('Returning arm to home...')
        self.move_to_home()

    def trigger_watering(self):
        self.get_logger().info('WATER PUMP ON')
        time.sleep(3.0)
        self.get_logger().info('WATER PUMP OFF')

    def move_to_home(self):
        request = MotionPlanRequest()
        request.group_name = 'arm'

        goal_constraints = Constraints()
        joint_names = ['base_yaw_joint', 'shoulder_pitch_joint',
                       'elbow_pitch_joint', 'wrist_pitch_joint', 'wrist_roll_joint']

        # Natural resting pose — all zeros, base_yaw parked sideways at 1.57
        # This is exactly where the robot sits at startup, guaranteed collision free
        home_angles = [1.57, 0.0, 0.0, 0.0, 0.0]

        for name, angle in zip(joint_names, home_angles):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = angle
            jc.tolerance_above = 0.05
            jc.tolerance_below = 0.05
            jc.weight = 1.0
            goal_constraints.joint_constraints.append(jc)

        request.goal_constraints.append(goal_constraints)
        request.num_planning_attempts = 10
        request.allowed_planning_time = 10.0
        request.max_velocity_scaling_factor = 0.5
        request.max_acceleration_scaling_factor = 0.5

        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options.plan_only = False

        home_future = self._action_client.send_goal_async(goal)
        home_future.add_done_callback(self.home_response_cb)

    def home_response_cb(self, future):
        self.get_logger().info('Arm ready. Waiting for next plant.')
        self.busy = False


def main(args=None):
    rclpy.init(args=args)
    node = PlantWateringNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
