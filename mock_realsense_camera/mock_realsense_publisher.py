import os
import cv2
import yaml
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory


class MockRealsensePublisher(Node):
    def __init__(self):
        super().__init__('mock_realsense_publisher')

        # Declare parameters
        self.declare_parameter('rgb_folder', 'data/rgb')
        self.declare_parameter('depth_folder', 'data/depth')
        self.declare_parameter('camera_info_path', 'config/realsense_camera_info.yaml')
        self.declare_parameter('image_rate', 10.0)

        # Resolve package share directory
        pkg_share = get_package_share_directory('mock_realsense_camera_2')

        # Default paths inside installed package
        default_rgb_folder = os.path.join(pkg_share, 'data', 'rgb')
        default_depth_folder = os.path.join(pkg_share, 'data', 'depth')
        default_camera_info_path = os.path.join(pkg_share, 'config', 'realsense_camera_info.yaml')

        # Retrieve parameter values
        rgb_param = self.get_parameter('rgb_folder').get_parameter_value().string_value
        depth_param = self.get_parameter('depth_folder').get_parameter_value().string_value
        info_param = self.get_parameter('camera_info_path').get_parameter_value().string_value

        # Resolve final paths — prefer absolute/existing paths, else fallback to package data
        self.rgb_folder = rgb_param if os.path.isabs(rgb_param) or os.path.exists(rgb_param) else default_rgb_folder
        self.depth_folder = depth_param if os.path.isabs(depth_param) or os.path.exists(depth_param) else default_depth_folder
        self.camera_info_path = info_param if os.path.isabs(info_param) or os.path.exists(info_param) else default_camera_info_path

        # Rate
        self.image_rate = self.get_parameter('image_rate').get_parameter_value().double_value

        # Log what was resolved
        self.get_logger().info(f"RGB folder: {self.rgb_folder}")
        self.get_logger().info(f"Depth folder: {self.depth_folder}")
        self.get_logger().info(f"Camera info: {self.camera_info_path}")

        # Publishers
        self.rgb_pub = self.create_publisher(Image, '/camera/color/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, '/camera/aligned_depth_to_color/image_raw', 10)
        self.rgb_info_pub = self.create_publisher(CameraInfo, '/camera/color/camera_info', 10)
        self.depth_info_pub = self.create_publisher(CameraInfo, '/camera/aligned_depth_to_color/camera_info', 10)

        self.bridge = CvBridge()

        # Load image file lists
        self.rgb_images = sorted([f for f in os.listdir(self.rgb_folder) if f.endswith('.png')])
        self.depth_images = sorted([f for f in os.listdir(self.depth_folder) if f.endswith('.png')])

        if len(self.rgb_images) != len(self.depth_images):
            self.get_logger().warn(
                f"RGB ({len(self.rgb_images)}) and depth ({len(self.depth_images)}) counts differ. "
                "Will use the smallest length."
            )

        self.num_frames = min(len(self.rgb_images), len(self.depth_images))
        self.index = 0

        # Load camera info
        self.rgb_info, self.depth_info = self.load_camera_info()

        # Timer for publishing synchronized pairs
        self.timer = self.create_timer(1.0 / self.image_rate, self.publish_pair)

        self.get_logger().info(f'Publishing {self.num_frames} synchronized RGB/depth pairs at {self.image_rate} Hz.')

    def load_camera_info(self):
        """Load CameraInfo from YAML file."""
        if not os.path.exists(self.camera_info_path):
            self.get_logger().error(f"Camera info file not found: {self.camera_info_path}")
            return CameraInfo(), CameraInfo()

        with open(self.camera_info_path, 'r') as f:
            cam_info_dict = yaml.safe_load(f)

        rgb_info = CameraInfo()
        depth_info = CameraInfo()

        for key, val in cam_info_dict.items():
            if hasattr(rgb_info, key):
                setattr(rgb_info, key, val)
                setattr(depth_info, key, val)

        rgb_info.header.frame_id = 'camera_color_optical_frame'
        depth_info.header.frame_id = 'camera_color_optical_frame'  # aligned depth shares same frame

        return rgb_info, depth_info

    def publish_pair(self):
        """Publish synchronized RGB + depth + camera info messages."""
        if self.num_frames == 0:
            self.get_logger().warn("No image pairs found in data folders.")
            return

        rgb_path = os.path.join(self.rgb_folder, self.rgb_images[self.index])
        depth_path = os.path.join(self.depth_folder, self.depth_images[self.index])

        rgb_frame = cv2.imread(rgb_path, cv2.IMREAD_COLOR)
        depth_frame = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)

        if rgb_frame is None or depth_frame is None:
            self.get_logger().error(f"Failed to read image pair index {self.index}. Skipping...")
            self.index = (self.index + 1) % self.num_frames
            return

        now = self.get_clock().now().to_msg()

        # Publish RGB image
        rgb_msg = self.bridge.cv2_to_imgmsg(rgb_frame, encoding='bgr8')
        rgb_msg.header.stamp = now
        rgb_msg.header.frame_id = 'camera_color_optical_frame'
        self.rgb_pub.publish(rgb_msg)

        # Publish aligned depth image
        depth_msg = self.bridge.cv2_to_imgmsg(depth_frame, encoding='16UC1')
        depth_msg.header.stamp = now
        depth_msg.header.frame_id = 'camera_color_optical_frame'
        self.depth_pub.publish(depth_msg)

        # Publish synchronized CameraInfo
        self.rgb_info.header.stamp = now
        self.depth_info.header.stamp = now
        self.rgb_info_pub.publish(self.rgb_info)
        self.depth_info_pub.publish(self.depth_info)

        self.get_logger().debug(f"Published synchronized frame {self.index}")
        self.index = (self.index + 1) % self.num_frames


def main(args=None):
    rclpy.init(args=args)
    node = MockRealsensePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
