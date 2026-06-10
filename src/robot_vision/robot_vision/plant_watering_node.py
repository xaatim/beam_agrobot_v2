import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PointStamped, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    PositionConstraint,
    JointConstraint,
    BoundingVolume,
)
from shape_msgs.msg import SolidPrimitive
import time

class PlantWateringNode(Node):

    def __init__(self):
        super().__init__('plant_watering_node')

        self._action_client = ActionClient(
            self,
            MoveGroup,
            'move_action' 
        )

        self.plant_sub = self.create_subscription(
            PointStamped,
            '/agrobot/target_watering_point',
            self.plant_detection_callback,
            10
        )

        self.busy = False
        self.get_logger().info('Plant watering node ready. Waiting for targets...')

    def plant_detection_callback(self, msg):
        if self.busy:
            return

        self.get_logger().info(f'Target received at X:{msg.point.x:.3f} Y:{msg.point.y:.3f} Z:{msg.point.z:.3f}')
        self.busy = True
        self.move_arm_to_plant(msg)

    def move_arm_to_plant(self, point_msg):
        self.get_logger().info('Waiting for move_action server...')
        self._action_client.wait_for_server()

        # Build the target pose
        target_pose = PoseStamped()
        target_pose.header = point_msg.header 
        
        target_pose.pose.position.x = point_msg.point.x
        target_pose.pose.position.y = point_msg.point.y
        target_pose.pose.position.z = point_msg.point.z - 0.15 
        
        target_pose.pose.orientation.w = 1.0

        position_constraint = PositionConstraint()
        position_constraint.header = point_msg.header
        position_constraint.link_name = 'camera_link'

        bounding_volume = BoundingVolume()
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [0.05] 
        bounding_volume.primitives.append(primitive) # type: ignore
        bounding_volume.primitive_poses.append(target_pose.pose) # type: ignore
        position_constraint.constraint_region = bounding_volume
        position_constraint.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(position_constraint) # type: ignore

        request = MotionPlanRequest()
        request.group_name = 'arm'
        request.goal_constraints.append(constraints) # type: ignore
        request.num_planning_attempts = 10
        request.allowed_planning_time = 5.0
        request.max_velocity_scaling_factor = 0.3
        request.max_acceleration_scaling_factor = 0.3

        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3

        self.get_logger().info('Sending TRAC-IK Cartesian request...')
        send_goal_future = self._action_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('MoveIt rejected the goal. Target is likely out of reach physically.')
            self.busy = False
            return
        
        self.get_logger().info('MoveIt accepted goal — arm is moving to plant!')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        status = future.result().result.error_code.val

        if status == 1:
            self.get_logger().info('SUCCESS: Arm reached the standoff point.')
            self.trigger_watering()
        else:
            self.get_logger().error(f'MoveIt failed mid-trajectory with error code: {status}')

        self.get_logger().info('Folding arm back to home position...')
        self.move_to_home()

    def trigger_watering(self):
        self.get_logger().info('💧 WATER PUMP ON 💧')
        time.sleep(3.0)
        self.get_logger().info('💧 WATER PUMP OFF 💧')

    def move_to_home(self):
        request = MotionPlanRequest()
        request.group_name = 'arm'
        
        goal_constraints = Constraints()
        joint_names = ['base_yaw_joint', 'shoulder_pitch_joint', 'elbow_pitch_joint', 'wrist_pitch_joint', 'wrist_roll_joint']
        home_angles = [0.0, -1.0, 1.0, 0.0, 0.0] 
        
        for name, angle in zip(joint_names, home_angles):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = angle
            jc.tolerance_above = 0.05
            jc.tolerance_below = 0.05
            jc.weight = 1.0
            goal_constraints.joint_constraints.append(jc) # type: ignore
            
        request.goal_constraints.append(goal_constraints) # type: ignore
        request.num_planning_attempts = 5
        request.allowed_planning_time = 3.0
        request.max_velocity_scaling_factor = 0.5
        request.max_acceleration_scaling_factor = 0.5

        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options.plan_only = False

        home_future = self._action_client.send_goal_async(goal)
        home_future.add_done_callback(self.home_response_cb)

    def home_response_cb(self, future):
        self.get_logger().info('Arm successfully returned home. Ready for next plant.')
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