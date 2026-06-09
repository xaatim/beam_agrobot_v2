import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import message_filters
import numpy as np
from ultralytics import YOLOE


class CropDetectorNode(Node):
    def __init__(self):
        super().__init__('crop_detector_node')
        self.bridge = CvBridge()
        self.model = YOLOE("yoloe-26s-seg.pt")
        self.target_crops = ["tomato plant",
                            "tomato", "chili plant", "lettuce"]
        self.model.set_classes(self.target_crops)

        self.target_pub = self.create_publisher(
            PointStamped,
            '/agrobot/target_watering_point',
            10
        )

        self.info_sub = self.create_subscription(
            CameraInfo,
            '/camera/depth/camera_info',
            self.camera_info_callback,
            10
        )
        self.intrinsics_loaded = False
        self.fx = 0.0
        self.fy = 0.0
        self.cx = 0.0
        self.cy = 0.0

        self.rgb_sub = message_filters.Subscriber(
            self, Image, '/camera/color/image_raw')
        self.depth_sub = message_filters.Subscriber(
            self, Image, '/camera/depth/image_raw')

        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub],
            queue_size=10,
            slop=0.1
        )
        self.sync.registerCallback(self.synchronized_camera_callback)
        self.get_logger().info("Agrobot Crop Detector Node Initialized and Waiting for Streams...")

    def camera_info_callback(self, msg):
        self.fx = msg.k[0]
        self.cx = msg.k[2]
        self.fy = msg.k[4]
        self.cy = msg.k[5]
        self.intrinsics_loaded = True
        self.destroy_subscription(self.info_sub)
        self.get_logger().info(
            f"Loaded Intrinsics -> fx: {self.fx}, fy: {self.fy}, cx: {self.cx}, cy: {self.cy}")

    def synchronized_camera_callback(self, rgb_msg, depth_msg):
        if not self.intrinsics_loaded:
            return

        cv_rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        cv_depth = self.bridge.imgmsg_to_cv2(
            depth_msg, desired_encoding='32FC1')

        results = self.model.predict(cv_rgb, conf=0.25, verbose=False)

        for result in results:
            if result.masks is not None and result.boxes is not None:
                num_boxes = len(result.boxes)
                for i in range(num_boxes):

                    class_id = int(result.boxes.cls[i].item())
                    label = self.target_crops[class_id] if class_id < len(
                        self.target_crops) else "unknown"

                    mask_np = result.masks.data[i].cpu().numpy()
                    y_indices, x_indices = np.where(mask_np > 0)

                    if len(y_indices) > 0:
                        u = int(np.mean(x_indices))
                        v = int(np.mean(y_indices))

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
                            f"Detected: {label} at X:{X:.3f}m, Y:{Y:.3f}m, Z:{Z:.3f}m")


def main(args=None):
    rclpy.init(args=args)
    node = CropDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
