from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge


class ImgNode(Node):
    def __init__(self):
        super().__init__('img_node')
        self.bridge = CvBridge()
        self.color_frame = None
        self.color_frame_stamp = None
        self.depth_frame = None
        self.intrinsics = None
        self.color_subscription = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.color_callback, 10)
        self.depth_subscription = self.create_subscription(
            Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)
        self.camera_info_subscription = self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info', self.camera_info_callback, 10)
        self.get_logger().info("Waiting for client's call...")

    def camera_info_callback(self, msg):
        self.intrinsics = {"fx": msg.k[0], "fy": msg.k[4], "ppx": msg.k[2], "ppy": msg.k[5]}

    def color_callback(self, msg):
        self.color_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.color_frame_stamp = str(msg.header.stamp.sec) + str(msg.header.stamp.nanosec)

    def depth_callback(self, msg):
        self.depth_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def get_color_frame(self):
        return self.color_frame

    def get_color_frame_stamp(self):
        return self.color_frame_stamp

    def get_depth_frame(self):
        return self.depth_frame

    def get_camera_intrinsic(self):
        return self.intrinsics
