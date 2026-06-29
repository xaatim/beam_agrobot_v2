import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint
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

ROBOT_HEADING = 0.0  
PARK_STANDOFF = 0.50 

LEFT_PARK_Y  =  1.5 - PARK_STANDOFF   #  1.0
RIGHT_PARK_Y = -1.5 + PARK_STANDOFF   # -1.0
TOMATO_Z = 0.66
WAYPOINTS = [

    (1.5, LEFT_PARK_Y,  1.57, 1.5,  1.5),
    (2.5, LEFT_PARK_Y,  1.57, 2.5,  1.5),
    (3.5, LEFT_PARK_Y,  1.57, 3.5,  1.5),
    (3.5, RIGHT_PARK_Y, -1.57, 3.5, -1.5),
    (2.5, RIGHT_PARK_Y, -1.57, 2.5, -1.5),
    (1.5, RIGHT_PARK_Y, -1.57, 1.5, -1.5),
]


class PlantWateringNode(Node):

    def __init__(self):
        super().__init__('plant_watering_node')

        self._arm_client = ActionClient(self, MoveGroup, 'move_action')
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.waypoint_index = 0
        self.state = 'INIT'
        self.busy = True

        self.current_crop_x = 0.0
        self.current_crop_y = 0.0
        self.current_crop_z = TOMATO_Z
        self.WATER_SHOULDER = 0.8
        self.WATER_ELBOW    = -1.2
        self.WATER_WRIST    = 0.5

        self.SCAN_SHOULDER  = 0.4
        self.SCAN_ELBOW     = -0.4
        self.SCAN_WRIST     = 0.3

        self.get_logger().info('Agrobot V5 — hardcoded waypoint mode. Starting...')
        self.create_timer(2.0, self._init)

    def _init(self):
        self.destroy_timer(list(self._timers)[0])
        self._arm_client.wait_for_server()
        self._nav_client.wait_for_server()
        self.get_logger().info('Servers ready. Starting waypoint sequence.')
        self._next_waypoint()

    def _next_waypoint(self):
        if self.waypoint_index >= len(WAYPOINTS):
            self.get_logger().info('ALL CROPS WATERED. Mission complete.')
            return

        park_x, park_y, scan_yaw, crop_x, crop_y = WAYPOINTS[self.waypoint_index]
        self.get_logger().info(
            f'=== Waypoint {self.waypoint_index + 1}/{len(WAYPOINTS)} '
            f'→ park at ({park_x}, {park_y:.2f}) ==='
        )

        self.current_scan_yaw  = scan_yaw
        self.current_crop_x    = crop_x
        self.current_crop_y    = crop_y
        self.current_crop_z    = TOMATO_Z

        if self.waypoint_index == 3:
            self.get_logger().info('Column 1 done. Executing U-turn to column 2...')
            self._nav_to_uturn_position(park_x, park_y)
        else:
            self._nav_to_park(park_x, park_y)

    def _nav_to_park(self, park_x, park_y):
        """Drive base to exact parking spot, robot always faces +X."""
        self.get_logger().info(f'Navigating to park position ({park_x}, {park_y:.2f})...')
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = park_x
        goal_pose.pose.position.y = park_y
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation = yaw_to_quaternion(ROBOT_HEADING)
        self._send_nav(goal_pose, 'PARK')

    def _nav_to_uturn_position(self, park_x, park_y):
        """
        U-turn: robot needs to end up at the first right-column
        waypoint, but facing -X (back down the aisle).
        After the U-turn the robot faces -X so heading = pi.
        Right column waypoints are ordered R3→R2→R1 (decreasing X)
        so the robot always drives forward (in its new -X direction).
        """
        self.get_logger().info('U-turn nav: driving to right column start...')
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = park_x
        goal_pose.pose.position.y = park_y
        goal_pose.pose.position.z = 0.0
        # Face -X after U-turn
        goal_pose.pose.orientation = yaw_to_quaternion(math.pi)
        self._send_nav(goal_pose, 'UTURN')

    def _send_nav(self, pose, label):
        self.nav_label = label
        goal = NavigateToPose.Goal()
        goal.pose = pose
        fut = self._nav_client.send_goal_async(goal)
        fut.add_done_callback(self._nav_goal_cb)

    def _nav_goal_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error(f'Nav2 rejected goal [{self.nav_label}]')
            return
        handle.get_result_async().add_done_callback(self._nav_result_cb)

    def _nav_result_cb(self, future):
        status = future.result().status
        if status != 4:
            self.get_logger().error(
                f'Nav2 FAILED (status {status}) [{self.nav_label}]'
            )
            return

        if self.nav_label == 'PARK':
            self.get_logger().info('Parked. Moving arm to scan pose...')
            self._move_arm_scan()

        elif self.nav_label == 'UTURN':
            self.get_logger().info('U-turn complete. Moving arm to scan pose...')
            self._move_arm_scan()

        elif self.nav_label == 'ADVANCE':
            self.get_logger().info('Advanced. Moving to next waypoint...')
            self.waypoint_index += 1
            self._next_waypoint()

    def _move_arm_scan(self):
        """Rotate arm to scan_yaw with light scan angles."""
        self.get_logger().info(
            f'Arm to scan pose at yaw={self.current_scan_yaw:.2f}...'
        )
        angles = [
            self.current_scan_yaw,
            self.SCAN_SHOULDER,
            self.SCAN_ELBOW,
            self.SCAN_WRIST,
            0.0
        ]
        self._send_arm(angles, 'SCAN')


    def _move_arm_water(self):
        """
        Compute yaw from base_link to crop then apply watering angles.
        Even though we know the exact crop position, we still compute
        yaw dynamically from TF so small parking errors are compensated.
        """
        try:
            map_pt = PointStamped()
            map_pt.header.frame_id = 'map'
            map_pt.header.stamp = self.get_clock().now().to_msg()
            map_pt.point.x = self.current_crop_x
            map_pt.point.y = self.current_crop_y
            map_pt.point.z = self.current_crop_z

            tf = self.tf_buffer.lookup_transform(
                'base_link', 'map', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            pt_base = tf2_geometry_msgs.do_transform_point(map_pt, tf)
        except TransformException as e:
            self.get_logger().error(f'TF error for arm water: {e}')
            return

        target_yaw = math.atan2(pt_base.point.y, pt_base.point.x)
        self.get_logger().info(
            f'Arm watering: yaw={target_yaw:.2f} '
            f'shoulder={self.WATER_SHOULDER} '
            f'elbow={self.WATER_ELBOW} '
            f'wrist={self.WATER_WRIST}'
        )

        angles = [
            target_yaw,
            self.WATER_SHOULDER,
            self.WATER_ELBOW,
            self.WATER_WRIST,
            0.0
        ]
        self._send_arm(angles, 'WATER')

    def _send_arm(self, joint_angles, label):
        self.arm_label = label
        joint_names = [
            'base_yaw_joint', 'shoulder_pitch_joint',
            'elbow_pitch_joint', 'wrist_pitch_joint', 'wrist_roll_joint'
        ]

        request = MotionPlanRequest()
        request.group_name = 'arm'
        constraints = Constraints()

        for name, angle in zip(joint_names, joint_angles):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = angle
            jc.tolerance_above = 0.05
            jc.tolerance_below = 0.05
            jc.weight = 1.0
            constraints.joint_constraints.append(jc) # type: ignore

        request.goal_constraints.append(constraints) # type: ignore
        request.num_planning_attempts = 10
        request.allowed_planning_time = 5.0
        request.max_velocity_scaling_factor = 0.4
        request.max_acceleration_scaling_factor = 0.4

        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options.plan_only = False

        fut = self._arm_client.send_goal_async(goal)
        fut.add_done_callback(self._arm_goal_cb)

    def _arm_goal_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error(f'MoveIt rejected arm goal [{self.arm_label}]')
            return
        handle.get_result_async().add_done_callback(self._arm_result_cb)

    def _arm_result_cb(self, future):
        status = future.result().result.error_code.val

        if self.arm_label == 'SCAN':
            if status == 1:
                self.get_logger().info('Scan pose reached. Extending arm to water...')
                self._move_arm_water()
            else:
                self.get_logger().error(f'Scan pose failed (code {status}). Skipping crop.')
                self._retract_and_advance()

        elif self.arm_label == 'WATER':
            if status == 1:
                self.get_logger().info('Arm at water position. Pumping...')
                self._pump()
            else:
                self.get_logger().error(f'Water pose failed (code {status}). Skipping crop.')
            self._retract_and_advance()

        elif self.arm_label == 'RETRACT':
            self.get_logger().info('Arm retracted. Advancing to next waypoint...')
            self._advance_to_next()

    def _pump(self):
        self.get_logger().info('>>> WATER PUMP ON <<<')
        time.sleep(3.0)
        self.get_logger().info('>>> WATER PUMP OFF <<<')

    def _retract_and_advance(self):
        """Return arm to scan pose (same yaw, scan angles) before moving base."""
        self.get_logger().info('Retracting arm to scan pose...')
        angles = [
            self.current_scan_yaw,
            self.SCAN_SHOULDER,
            self.SCAN_ELBOW,
            self.SCAN_WRIST,
            0.0
        ]
        self._send_arm(angles, 'RETRACT')

    def _advance_to_next(self):
        """Increment waypoint index and go."""
        self.waypoint_index += 1
        self._next_waypoint()


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