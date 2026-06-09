import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint

class ArmTest(Node):
    def __init__(self):
        super().__init__('arm_test_node')
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.client = ActionClient(self, MoveGroup, 'move_action')
        self.goal_sent = False
        self.timer = self.create_timer(3.0, self.fire_goal)
        self.get_logger().info('Arm test spinning up... sending goal in 3 secs.')

    def fire_goal(self):
        if self.goal_sent:
            return
        self.goal_sent = True
        self.timer.cancel()

        self.get_logger().info('Hunting for move_action server...')
        self.client.wait_for_server()

        # Build joint constraints instead of XYZ Cartesian constraints
        goal_constraints = Constraints()
        
        # A simple, safe pose
        joint_names = [
            'base_yaw_joint', 
            'shoulder_pitch_joint', 
            'elbow_pitch_joint', 
            'wrist_pitch_joint', 
            'wrist_roll_joint'
        ]
        target_angles = [0.5, -0.4, 0.5, 0.0, 0.0] 

        for i in range(len(joint_names)):
            jc = JointConstraint()
            jc.joint_name = joint_names[i]
            jc.position = target_angles[i]
            jc.tolerance_above = 0.05
            jc.tolerance_below = 0.05
            jc.weight = 1.0
            goal_constraints.joint_constraints.append(jc)

        req = MotionPlanRequest()
        req.group_name = 'arm'
        req.goal_constraints.append(goal_constraints)
        req.num_planning_attempts = 10
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = 0.3
        req.max_acceleration_scaling_factor = 0.3

        goal_msg = MoveGroup.Goal()
        goal_msg.request = req
        goal_msg.planning_options.plan_only = False
        goal_msg.planning_options.replan = True
        goal_msg.planning_options.replan_attempts = 3

        self.get_logger().info('Sending Joint-Space goal to bypass IK...')
        
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
            self.get_logger().info('SUCCESS! Target reached.')
        else:
            self.get_logger().error(f'FAILED. Error code: {status}. Try tweaking the target angles.')

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