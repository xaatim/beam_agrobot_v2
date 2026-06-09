import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, BoundingVolume
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive

class ArmTest(Node):
    def __init__(self):
        super().__init__('arm_test_node')
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.client = ActionClient(self, MoveGroup, 'move_action')
        self.goal_sent = False
        self.timer = self.create_timer(3.0, self.fire_goal)
        self.get_logger().info('Arm test spinning up... sending Cartesian goal in 3 secs.')

    def fire_goal(self):
        if self.goal_sent:
            return
        self.goal_sent = True
        self.timer.cancel()

        self.get_logger().info('Hunting for move_action server...')
        self.client.wait_for_server()

        # Cartesian XYZ Target (Simulating a YOLOE detection coordinate)
        target_x = 0.4
        target_y = 0.0
        target_z = 0.5

        pose = PoseStamped()
        pose.header.frame_id = 'base_link'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = target_x
        pose.pose.position.y = target_y
        pose.pose.position.z = target_z
        pose.pose.orientation.w = 1.0

        # We give the solver a 5cm sphere to aim for
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.05]

        vol = BoundingVolume()
        vol.primitives.append(sphere) # type: ignore
        vol.primitive_poses.append(pose.pose) # type: ignore

        # Tie the constraint to your camera link
        constraint = PositionConstraint()
        constraint.header.frame_id = 'base_link'
        constraint.link_name = 'camera_link'
        constraint.constraint_region = vol
        constraint.weight = 1.0

        goal_constraints = Constraints()
        goal_constraints.position_constraints.append(constraint) # type: ignore

        req = MotionPlanRequest()
        req.group_name = 'arm'
        req.goal_constraints.append(goal_constraints) # type: ignore
        
        # Give TRAC-IK plenty of attempts and time to solve the math
        req.num_planning_attempts = 10
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = 0.3
        req.max_acceleration_scaling_factor = 0.3

        goal_msg = MoveGroup.Goal()
        goal_msg.request = req
        goal_msg.planning_options.plan_only = False
        goal_msg.planning_options.replan = True
        goal_msg.planning_options.replan_attempts = 3

        self.get_logger().info(f'Sending Cartesian goal to X:{target_x} Y:{target_y} Z:{target_z}')
        
        self.future = self.client.send_goal_async(goal_msg)
        self.future.add_done_callback(self.goal_response_cb)
        
    def goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('MoveIt rejected the goal. Check your limits or collisions.')
            return
        
        self.get_logger().info('Goal accepted! Watch Gazebo, it should be moving.')
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.result_cb)

    def result_cb(self, future):
        status = future.result().result.error_code.val
        if status == 1:
            self.get_logger().info('SUCCESS! TRAC-IK solved the Cartesian target.')
        else:
            self.get_logger().error(f'FAILED. Error code: {status}. Try tweaking the XYZ target.')

def main(args=None):
    rclpy.init(args=args)
    node = ArmTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()