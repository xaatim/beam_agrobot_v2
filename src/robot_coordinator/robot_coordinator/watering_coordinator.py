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


# =======================================================
# HARDCODED WAYPOINTS FROM WORLD FILE
#
# Robot is 0.66m wide (wheel center to wheel center = 0.664m)
# Pot radius = 0.12m, positioned at y = ±1.5
# Safe standoff from robot CENTER to pot CENTER:
#   half_robot_width(0.33) + pot_radius(0.12) + clearance(0.05) = 0.50m
# So robot center parks at y = 1.5 - 0.50 = 1.0 (left column)
#                             y = -1.5 + 0.50 = -1.0 (right column)
#
# Robot faces forward (yaw=0) the entire time — never rotates.
# Arm sweeps left (+1.57) for left column, right (-1.57) for right.
#
# Tomato z = 0.66m above ground.
# Arm base is at roughly z=0.28m + robot body height.
# The arm needs to reach down-and-sideways to z=0.66.
#
# TUNE: if robot still clips pots, increase PARK_STANDOFF.
#       if arm can't reach, decrease PARK_STANDOFF.
# =======================================================

ROBOT_HEADING = 0.0   # always faces +X direction (forward down the aisle)
PARK_STANDOFF = 0.50  # distance from robot center to pot center laterally

# Left column pots at y=+1.5, robot parks at y = 1.5 - PARK_STANDOFF
LEFT_PARK_Y  =  1.5 - PARK_STANDOFF   #  1.0
RIGHT_PARK_Y = -1.5 + PARK_STANDOFF   # -1.0

# Tomato world z for arm targeting
TOMATO_Z = 0.66

# Each waypoint: (park_x, park_y, arm_scan_yaw, crop_map_x, crop_map_y)
# park_x/y   = where robot base_link center should be when watering
# arm_scan_yaw = direction arm faces to scan (+1.57 left, -1.57 right)
# crop_map_x/y = exact tomato position for arm IK targeting
WAYPOINTS = [
    # --- LEFT COLUMN (y = +1.5), robot parks at y = +1.0 ---
    # L1
    (1.5, LEFT_PARK_Y,  1.57, 1.5,  1.5),
    # L2
    (2.5, LEFT_PARK_Y,  1.57, 2.5,  1.5),
    # L3
    (3.5, LEFT_PARK_Y,  1.57, 3.5,  1.5),
    # --- RIGHT COLUMN (y = -1.5), robot parks at y = -1.0 ---
    # R1 — note: robot now faces -X (drove up col1, U-turns, drives back down)
    # After U-turn robot faces -X so we approach R3 first then R2 then R1
    # R3
    (3.5, RIGHT_PARK_Y, -1.57, 3.5, -1.5),
    # R2
    (2.5, RIGHT_PARK_Y, -1.57, 2.5, -1.5),
    # R1
    (1.5, RIGHT_PARK_Y, -1.57, 1.5, -1.5),
]


class PlantWateringNode(Node):

    def __init__(self):
        super().__init__('plant_watering_node')

        self._arm_client = ActionClient(self, MoveGroup, 'move_action')
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # -------------------------------------------------------
        # WAYPOINT STATE MACHINE
        # We walk through WAYPOINTS in order. Each step:
        #   1. NAV  → drive base to (park_x, park_y) heading=0
        #   2. SCAN → rotate arm to scan_yaw (so camera confirms crop)
        #   3. WATER → extend arm to watering angles
        #   4. PUMP → water for 3 seconds
        #   5. RETRACT → return arm to scan pose
        #   6. next waypoint
        # Between col1 and col2 there is a U-turn nav step.
        # -------------------------------------------------------
        self.waypoint_index = 0
        self.state = 'INIT'
        self.busy = True

        # Current crop target for arm IK (set from WAYPOINTS)
        self.current_crop_x = 0.0
        self.current_crop_y = 0.0
        self.current_crop_z = TOMATO_Z

        # -------------------------------------------------------
        # ARM ANGLES — tune these if end effector misses the crop
        #
        # WATER pose: shoulder lifts arm up, elbow folds it over,
        # wrist tips end effector downward toward the tomato.
        # Start with these values and adjust:
        #   shoulder too low → increase WATER_SHOULDER toward 1.0
        #   arm not reaching far enough → decrease WATER_ELBOW (more negative)
        #   tip pointing wrong way → adjust WATER_WRIST
        # -------------------------------------------------------
        self.WATER_SHOULDER = 0.8
        self.WATER_ELBOW    = -1.2
        self.WATER_WRIST    = 0.5

        # SCAN pose: lighter position, arm held sideways to see crops
        self.SCAN_SHOULDER  = 0.4
        self.SCAN_ELBOW     = -0.4
        self.SCAN_WRIST     = 0.3

        self.get_logger().info('Agrobot V5 — hardcoded waypoint mode. Starting...')
        self.create_timer(2.0, self._init)

    # =======================================================
    # INIT
    # =======================================================

    def _init(self):
        # Only fires once
        self.destroy_timer(list(self._timers)[0])
        self._arm_client.wait_for_server()
        self._nav_client.wait_for_server()
        self.get_logger().info('Servers ready. Starting waypoint sequence.')
        self._next_waypoint()

    # =======================================================
    # WAYPOINT SEQUENCER
    # =======================================================

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

        # Check if this is the U-turn transition (col1→col2)
        # That happens between index 2 (L3) and index 3 (R3).
        # We detect it by a sign change in park_y vs previous waypoint.
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
        # Robot always faces forward along X axis — never rotates toward crop
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

    # =======================================================
    # NAV2
    # =======================================================

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
            # After U-turn heading is now -X, update ROBOT_HEADING
            # for subsequent advance moves (not needed since waypoints
            # are hardcoded, but good to log)
            self._move_arm_scan()

        elif self.nav_label == 'ADVANCE':
            self.get_logger().info('Advanced. Moving to next waypoint...')
            self.waypoint_index += 1
            self._next_waypoint()

    # =======================================================
    # ARM — SCAN POSE
    # =======================================================

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

    # =======================================================
    # ARM — WATER POSE
    # =======================================================

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

    # =======================================================
    # ARM — SEND GOAL
    # =======================================================

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
            constraints.joint_constraints.append(jc)

        request.goal_constraints.append(constraints)
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
            # Arrived at scan pose — now extend to water
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
            # Arm back to scan pose — advance to next waypoint
            self.get_logger().info('Arm retracted. Advancing to next waypoint...')
            self._advance_to_next()

    # =======================================================
    # PUMP
    # =======================================================

    def _pump(self):
        self.get_logger().info('>>> WATER PUMP ON <<<')
        time.sleep(3.0)
        self.get_logger().info('>>> WATER PUMP OFF <<<')

    # =======================================================
    # RETRACT ARM THEN ADVANCE
    # =======================================================

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