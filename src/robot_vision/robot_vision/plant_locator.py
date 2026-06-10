import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import message_filters
import numpy as np
import cv2


class CropDetectorNode(Node):
    def __init__(self):
        super().__init__('crop_detector_node')
        self.bridge = CvBridge()

        self.target_pub = self.create_publisher(
            PointStamped,
            '/agrobot/target_watering_point',
            10
        )

        self.info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera/depth/camera_info',
            self.camera_info_callback,
            10
        )
        self.intrinsics_loaded = False
        self.fx = 0.0
        self.fy = 0.0
        self.cx = 0.0
        self.cy = 0.0

        self.rgb_sub = message_filters.Subscriber(
            self, Image, '/camera/camera/image_raw')
        self.depth_sub = message_filters.Subscriber(
            self, Image, '/camera/camera/depth/image_raw')

        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub],
            queue_size=10,
            slop=0.1
        )
        self.sync.registerCallback(self.synchronized_camera_callback)
        self.get_logger().info("Agrobot Fast Sphere Detector Initialized...")

    def camera_info_callback(self, msg):
        self.fx = msg.k[0]
        self.cx = msg.k[2]
        self.fy = msg.k[4]
        self.cy = msg.k[5]
        self.intrinsics_loaded = True
        self.destroy_subscription(self.info_sub)
        self.get_logger().info(
            f"Loaded Intrinsics -> fx: {self.fx:.2f}, fy: {self.fy:.2f}, cx: {self.cx:.2f}, cy: {self.cy:.2f}")

    def synchronized_camera_callback(self, rgb_msg, depth_msg):
        if not self.intrinsics_loaded:
            return

        cv_rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        cv_depth = self.bridge.imgmsg_to_cv2(
            depth_msg, desired_encoding='32FC1')

        # Convert BGR to HSV for color tracking
        hsv = cv2.cvtColor(cv_rgb, cv2.COLOR_BGR2HSV)

        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        red_mask = cv2.bitwise_or(mask1, mask2)

        contours, _ = cv2.findContours(
            red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            area = cv2.contourArea(contour)

            if area > 200:
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    u = int(M["m10"] / M["m00"])
                    v = int(M["m01"] / M["m00"])

                    if v >= cv_depth.shape[0] or u >= cv_depth.shape[1]:
                        continue

                    Z = cv_depth[v, u]

                    if np.isnan(Z) or np.isinf(Z) or Z <= 0.05:
                        continue

                    X = (u - self.cx) * Z / self.fx
                    Y = (v - self.cy) * Z / self.fy

                    target_msg = PointStamped()
                    target_msg.header = rgb_msg.header
                    target_msg.point.x = float(X)
                    target_msg.point.y = float(Y)
                    target_msg.point.z = float(Z)

                    self.target_pub.publish(target_msg)
                    self.get_logger().info(
                        f"Target Locked at X:{X:.3f}m, Y:{Y:.3f}m, Z:{Z:.3f}m")


def main(args=None):
    rclpy.init(args=args)
    node = CropDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
