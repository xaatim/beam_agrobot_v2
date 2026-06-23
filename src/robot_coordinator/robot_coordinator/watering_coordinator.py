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


def yaw_to_quaternion(yaw):
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class PlantWateringNode(Node):

    def __init__(self):
        super().__init__('plant_watering_node')

        self._arm_action_client = ActionClient(self, MoveGroup, 'move_action')
        self._nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.plant_sub = self.create_subscription(
            PointStamped,
            '/agrobot/target_watering_point',
            self.plant_detection_callback,
            10
        )

        self.busy = True
        self.standoff_distance = 0.55  
        
        self.watered_plants = [] 
        self.scan_yaw = 1.57          
        self.time_without_plant = 0.0 
        self.is_recovering_from_water = False
        
        # Tracks what Nav2 is currently doing
        self.nav_state = 'IDLE'

        self.get_logger().info('Initializing Agrobot Rail-Walker V3...')
        self.timer = self.create_timer(2.0, self.init_pose)
        
        self.search_timer = self.create_timer(1.0, self.search_step_callback)

    def init_pose(self):
        self.timer.cancel()
        self._arm_action_client.wait_for_server()
        self.move_to_scan_pose(after_watering=False)

    # =======================================================
    # SWEEP & U-TURN LOGIC (NAV2 DRIVEN)
    # =======================================================

    def search_step_callback(self):
        if self.busy:
            return
            
        self.time_without_plant += 1.0
        
        if self.time_without_plant >= 4.0:
            self.time_without_plant = 0.0
            self.scan_yaw -= 0.26  
            
            if self.scan_yaw < 0.0:
                self.busy = True
                self.execute_u_turn()
            else:
                self.get_logger().info(f'Sweeping arm forward to {self.scan_yaw:.2f} rads...')
                self.busy = True
                self.move_to_scan_pose(after_watering=False)

    def execute_u_turn(self):
        self.get_logger().info('>>> END OF COLUMN. EXECUTING 180 U-TURN VIA NAV2 <<<')
        try:
            trans_robot = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            
            q = trans_robot.transform.rotation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            current_yaw = math.atan2(siny_cosp, cosy_cosp)
            
            # Add 180 degrees
            new_yaw = current_yaw + math.pi 
            
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.pose.position.x = trans_robot.transform.translation.x
            goal_pose.pose.position.y = trans_robot.transform.translation.y
            goal_pose.pose.orientation = yaw_to_quaternion(new_yaw)

            self.send_nav2_goal(goal_pose, 'U_TURN')
        except Exception as e:
            self.get_logger().error(f'TF Error on U-Turn: {e}')
            self.busy = False

    def advance_forward(self):
        self.get_logger().info('Arm clear. Advancing 40cm forward down aisle...')
        try:
            # Create a point 40cm straight ahead of the robot
            local_forward = PointStamped()
            local_forward.header.frame_id = 'base_link'
            local_forward.header.stamp = self.get_clock().now().to_msg()
            local_forward.point.x = 0.40 

            trans_robot = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            map_forward = tf2_geometry_msgs.do_transform_point(local_forward, trans_robot)

            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.pose.position.x = map_forward.point.x
            goal_pose.pose.position.y = map_forward.point.y
            goal_pose.pose.orientation = trans_robot.transform.rotation # Keep same heading

            self.send_nav2_goal(goal_pose, 'ADVANCE_FORWARD')
        except Exception as e:
            self.get_logger().error(f'TF Error advancing: {e}')
            self.busy = False


    # =======================================================
    # LANE CHANGE NAVIGATION
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

            for wx, wy in self.watered_plants:
                dist_to_watered = math.hypot(plant_map.point.x - wx, plant_map.point.y - wy)
                if dist_to_watered < 0.8: 
                    return

            trans_base = self.tf_buffer.lookup_transform(
                'base_link', msg.header.frame_id, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            plant_base_link = tf2_geometry_msgs.do_transform_point(msg, trans_base)

            trans_robot = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
        except TransformException:
            return

        # DISTANCE FILTER FIX: Ignore plants that are more than 1.2m away or behind the robot
        forward_shift = plant_base_link.point.x 
        if forward_shift > 1.2 or forward_shift < -0.1:
            return

        self.busy = True
        self.time_without_plant = 0.0  
        
        is_left = plant_base_link.point.y > 0
        self.scan_yaw = 1.57 if is_left else -1.57
        
        self.get_logger().info('Target locked! Calculating lane-change approach...')

        self.current_target_map_x = plant_map.point.x
        self.current_target_map_y = plant_map.point.y
        self.current_target_map_z = plant_map.point.z

        if is_left:
            lateral_shift = plant_base_link.point.y - self.standoff_distance
        else:
            lateral_shift = plant_base_link.point.y + self.standoff_distance

        if abs(forward_shift) < 0.15 and abs(lateral_shift) < 0.15:
            self.get_logger().info('Perfectly parked in lane. Deploying arm.')
            self.move_arm_to_plant()
        else:
            self.get_logger().info(f'Changing lanes: Forward {forward_shift:.2f}m, Sideways {lateral_shift:.2f}m...')

            local_parking_spot = PointStamped()
            local_parking_spot.header.frame_id = 'base_link'
            local_parking_spot.header.stamp = self.get_clock().now().to_msg()
            local_parking_spot.point.x = forward_shift
            local_parking_spot.point.y = lateral_shift
            local_parking_spot.point.z = 0.0

            map_parking_spot = tf2_geometry_msgs.do_transform_point(local_parking_spot, trans_robot)

            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.pose.position.x = map_parking_spot.point.x
            goal_pose.pose.position.y = map_parking_spot.point.y
            goal_pose.pose.orientation = trans_robot.transform.rotation

            self.send_nav2_goal(goal_pose, 'APPROACH_PLANT')

    def send_nav2_goal(self, pose, state_name):
        self._nav_action_client.wait_for_server()
        self.nav_state = state_name
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
            if self.nav_state == 'APPROACH_PLANT':
                self.get_logger().info('Lane change successful! Deploying arm...')
                self.move_arm_to_plant()
            elif self.nav_state == 'ADVANCE_FORWARD':
                self.get_logger().info('Advance complete. Resuming patrol...')
                self.busy = False
            elif self.nav_state == 'U_TURN':
                self.get_logger().info('U-Turn complete. Ready for Column 2...')
                self.scan_yaw = 1.57 # Always scan left after a U-Turn
                self.time_without_plant = 0.0
                self.move_to_scan_pose(after_watering=False)
        else:
            self.get_logger().error(f'Nav2 failed state: {self.nav_state}')
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

        # THE ARM FIX:
        # Reduced the pitch values significantly. 
        # If the arm still points up instead of down, change these to negative numbers (e.g. -0.15)
        target_angles = [target_yaw, 0.15, 0.15, 0.0, 0.0]

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

        self.get_logger().info(f'Deploying arm... Yaw: {target_yaw:.2f}')
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
        
        self.get_logger().info('Returning arm to scan pose...')
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
        if self.is_recovering_from_water:
            self.advance_forward()
        else:
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