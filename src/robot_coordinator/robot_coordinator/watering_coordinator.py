import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion, Twist
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    JointConstraint,
)
from nav2_msgs.action import NavigateToPose

import time
import math

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs


class PlantWateringNode(Node):

    def __init__(self):
        super().__init__('plant_watering_node')

        self._arm_action_client = ActionClient(self, MoveGroup, 'move_action')
        self._nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.plant_sub = self.create_subscription(
            PointStamped,
            '/agrobot/target_watering_point',
            self.plant_detection_callback,
            10
        )

        self.busy = True
        self.standoff_distance = 0.50  
        
        # MEMORY AND SCANNING
        self.watered_plants = [] 
        self.scan_yaw = 1.57          # Start at 90 degrees Left
        self.time_without_plant = 0.0 # Timer to trigger the sweep
        self.is_recovering_from_water = False

        self.get_logger().info('Initializing Agrobot Column Sweeper...')
        self.timer = self.create_timer(2.0, self.init_pose)
        
        # The Sweep Timer: Checks every 1 second if we need to sweep the arm forward
        self.search_timer = self.create_timer(1.0, self.search_step_callback)

    def init_pose(self):
        self.timer.cancel()
        self._arm_action_client.wait_for_server()
        self.move_to_scan_pose(after_watering=False)

    # =======================================================
    # SWEEP & U-TURN LOGIC
    # =======================================================

    def search_step_callback(self):
        if self.busy:
            return
            
        self.time_without_plant += 1.0
        
        # If 4 seconds pass and no plant is seen, sweep the arm forward 15 degrees
        if self.time_without_plant >= 4.0:
            self.time_without_plant = 0.0
            self.scan_yaw -= 0.26  # Subtract 15 degrees (~0.26 rads)
            
            if self.scan_yaw < 0.0:
                # Arm swept all the way to the front and saw nothing. End of column!
                self.busy = True
                self.execute_u_turn()
            else:
                self.get_logger().info(f'Sweeping arm forward to {self.scan_yaw:.2f} rads to look for crops...')
                self.busy = True
                self.move_to_scan_pose(after_watering=False)

    def execute_u_turn(self):
        self.get_logger().info('>>> END OF COLUMN. EXECUTING 180 U-TURN <<<')
        twist = Twist()
        
        # 1. Drive forward a tiny bit to clear the last plant
        twist.linear.x = 0.3
        for _ in range(15):
            self.cmd_pub.publish(twist)
            time.sleep(0.1)
            
        # 2. Turn 180 degrees (Rotate at 1.0 rad/s for ~3.14 seconds)
        twist.linear.x = 0.0
        twist.angular.z = 1.0 
        for _ in range(32): 
            self.cmd_pub.publish(twist)
            time.sleep(0.1)
            
        # 3. Stop
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)
        
        self.get_logger().info('U-Turn complete. Resetting arm to scan Column 2...')
        
        # Since we U-turned, the "Left" side of the robot now perfectly faces Column 2!
        self.scan_yaw = 1.57 
        self.time_without_plant = 0.0
        self.move_to_scan_pose(after_watering=False)


    # =======================================================
    # DETECTION & NAVIGATION LOGIC
    # =======================================================

    def plant_detection_callback(self, msg):
        if self.busy:
            return

        try:
            trans_map = self.tf_buffer.lookup_transform(
                'map', msg.header.frame_id, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            plant_map = tf2_geometry_msgs.do_transform_point(msg, trans_map)

            # Check Memory (The U-turn won't break this because memory is in absolute MAP coordinates)
            for wx, wy in self.watered_plants:
                dist_to_watered = math.hypot(plant_map.point.x - wx, plant_map.point.y - wy)
                if dist_to_watered < 0.8: 
                    return

            trans_robot = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
        except TransformException:
            return

        # NEW PLANT FOUND! Lock the state machine.
        self.busy = True
        self.time_without_plant = 0.0  # Reset the sweep timer
        self.scan_yaw = 1.57           # Reset the arm to look 90 degrees for the next plant
        
        self.get_logger().info('Target locked! Calculating aisle approach...')

        self.current_target_map_x = plant_map.point.x
        self.current_target_map_y = plant_map.point.y
        self.current_target_map_z = plant_map.point.z

        robot_x = trans_robot.transform.translation.x
        robot_y = trans_robot.transform.translation.y

        dx = plant_map.point.x - robot_x
        dy = plant_map.point.y - robot_y
        dist = math.hypot(dx, dy)

        if dist < (self.standoff_distance + 0.1):
            self.get_logger().info('Already beside the plant.')
            self.move_arm_to_plant()
        else:
            self.get_logger().info('Driving base alongside the crop row...')

            angle_to_plant = math.atan2(dy, dx)
            goal_x = plant_map.point.x - (self.standoff_distance * math.cos(angle_to_plant))
            goal_y = plant_map.point.y - (self.standoff_distance * math.sin(angle_to_plant))

            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            goal_pose.pose.position.x = goal_x
            goal_pose.pose.position.y = goal_y
            
            # KEEP FORWARD HEADING
            goal_pose.pose.orientation = trans_robot.transform.rotation

            self.send_nav2_goal(goal_pose)

    def send_nav2_goal(self, pose):
        self._nav_action_client.wait_for_server()
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        send_goal_future = self._nav_action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.nav_goal_response_cb)

    def nav_goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.busy = False
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav_result_cb)

    def nav_result_cb(self, future):
        result = future.result()
        if result.status == 4:
            self.get_logger().info('Base parked alongside crop! Deploying arm...')
            self.move_arm_to_plant()
        else:
            self.busy = False

    # =======================================================
    # ARM REACHING LOGIC
    # =======================================================

    def move_arm_to_plant(self):
        try:
            map_pt = PointStamped()
            map_pt.header.frame_id = 'map'
            map_pt.header.stamp = rclpy.time.Time().to_msg()
            map_pt.point.x = self.current_target_map_x
            map_pt.point.y = self.current_target_map_y
            map_pt.point.z = self.current_target_map_z

            transform = self.tf_buffer.lookup_transform(
                'base_link', 'map', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            base_link_point = tf2_geometry_msgs.do_transform_point(map_pt, transform)
        except TransformException:
            self.busy = False
            return

        self._arm_action_client.wait_for_server()

        target_yaw = math.atan2(base_link_point.point.y, base_link_point.point.x)

        request = MotionPlanRequest()
        request.group_name = 'arm'
        goal_constraints = Constraints()
        joint_names = ['base_yaw_joint', 'shoulder_pitch_joint',
                       'elbow_pitch_joint', 'wrist_pitch_joint', 'wrist_roll_joint']

        target_angles = [target_yaw, 0.2, 0.2, 0.0, 0.0]

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
        request.allowed_planning_time = 5.0
        request.max_velocity_scaling_factor = 0.4
        request.max_acceleration_scaling_factor = 0.4

        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options.plan_only = False

        self.get_logger().info(f'Deploying arm to crop... Yaw: {target_yaw:.2f}')
        send_goal_future = self._arm_action_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self.arm_goal_response_callback)

    def arm_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.busy = False
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.arm_result_callback)

    def arm_result_callback(self, future):
        status = future.result().result.error_code.val

        if status == 1:
            self.trigger_watering()
            self.watered_plants.append((self.current_target_map_x, self.current_target_map_y))
            self.get_logger().info(f'Plant marked. Total watered: {len(self.watered_plants)}')
        
        self.get_logger().info('Returning arm to 90 degrees...')
        self.move_to_scan_pose(after_watering=True)

    def trigger_watering(self):
        self.get_logger().info('>>> WATER PUMP ON <<<')
        time.sleep(3.0)
        self.get_logger().info('>>> WATER PUMP OFF <<<')

    def move_to_scan_pose(self, after_watering=False):
        self.is_recovering_from_water = after_watering
        request = MotionPlanRequest()
        request.group_name = 'arm'

        goal_constraints = Constraints()
        joint_names = ['base_yaw_joint', 'shoulder_pitch_joint',
                       'elbow_pitch_joint', 'wrist_pitch_joint', 'wrist_roll_joint']

        # Uses the dynamic scan_yaw to sweep forward!
        scan_angles = [self.scan_yaw, 0.4, -0.4, 0.3, 0.0]

        for name, angle in zip(joint_names, scan_angles):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = angle
            jc.tolerance_above = 0.05
            jc.tolerance_below = 0.05
            jc.weight = 1.0
            goal_constraints.joint_constraints.append(jc)

        request.goal_constraints.append(goal_constraints)
        request.num_planning_attempts = 10
        request.allowed_planning_time = 5.0
        request.max_velocity_scaling_factor = 0.5
        request.max_acceleration_scaling_factor = 0.5

        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options.plan_only = False

        home_future = self._arm_action_client.send_goal_async(goal)
        home_future.add_done_callback(self.scan_response_cb)

    def scan_response_cb(self, future):
        # ONLY drive forward if we just watered a plant
        if self.is_recovering_from_water:
            self.get_logger().info('Arm clear. Driving forward down aisle...')
            twist = Twist()
            twist.linear.x = 0.3 
            
            for _ in range(25):
                self.cmd_pub.publish(twist)
                time.sleep(0.1)
                
            twist.linear.x = 0.0
            self.cmd_pub.publish(twist)
        
        self.get_logger().info('Resuming patrol...')
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