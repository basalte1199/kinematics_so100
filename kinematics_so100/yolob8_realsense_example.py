import math
from typing import Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge

try:
	from ultralytics import YOLO
except Exception:  # pragma: no cover - handled by runtime log
	YOLO = None

try:
	from message_filters import ApproximateTimeSynchronizer, Subscriber
except Exception:  # pragma: no cover - handled by runtime log
	ApproximateTimeSynchronizer = None
	Subscriber = None


class YoloRgbdRealsenseNode(Node):
	def __init__(self) -> None:
		super().__init__("yolov8_realsense_example")

		self.declare_parameter("rgbd_topic", "/camera/rdgb")
		self.declare_parameter("rgb_topic", "")
		self.declare_parameter("depth_topic", "")
		self.declare_parameter("camera_info_topic", "/camera/camera_info")
		self.declare_parameter("target_class", "person")
		self.declare_parameter("model_path", "yolov8n.pt")
		self.declare_parameter("conf_threshold", 0.5)
		self.declare_parameter("depth_unit_scale", 0.001)
		self.declare_parameter("depth_window", 5)

		self.rgbd_topic = self.get_parameter("rgbd_topic").value
		self.rgb_topic = self.get_parameter("rgb_topic").value
		self.depth_topic = self.get_parameter("depth_topic").value
		self.camera_info_topic = self.get_parameter("camera_info_topic").value
		self.target_class = self.get_parameter("target_class").value
		self.model_path = self.get_parameter("model_path").value
		self.conf_threshold = float(self.get_parameter("conf_threshold").value)
		self.depth_unit_scale = float(self.get_parameter("depth_unit_scale").value)
		self.depth_window = int(self.get_parameter("depth_window").value)

		self.bridge = CvBridge()
		self.camera_info: Optional[CameraInfo] = None

		self.point_pub = self.create_publisher(PointStamped, "target_point", 10)

		if YOLO is None:
			self.get_logger().error("ultralytics が見つかりません。pipでインストールしてください。")
			self.model = None
		else:
			self.model = YOLO(self.model_path)

		self.create_subscription(
			CameraInfo, self.camera_info_topic, self.camera_info_callback, 10
		)

		self.use_separate_topics = bool(self.rgb_topic) and bool(self.depth_topic)
		if self.use_separate_topics:
			if ApproximateTimeSynchronizer is None or Subscriber is None:
				self.get_logger().error("message_filters が利用できません。")
				return

			rgb_sub = Subscriber(self, Image, self.rgb_topic)
			depth_sub = Subscriber(self, Image, self.depth_topic)
			sync = ApproximateTimeSynchronizer(
				[rgb_sub, depth_sub], queue_size=10, slop=0.2
			)
			sync.registerCallback(self.rgb_depth_callback)
			self.get_logger().info(
				f"Subscribed to rgb: {self.rgb_topic}, depth: {self.depth_topic}"
			)
		else:
			self.create_subscription(Image, self.rgbd_topic, self.rgbd_callback, 10)
			self.get_logger().info(f"Subscribed to rgbd: {self.rgbd_topic}")

	def camera_info_callback(self, msg: CameraInfo) -> None:
		self.camera_info = msg

	def rgbd_callback(self, msg: Image) -> None:
		if self.model is None:
			return
		if self.camera_info is None:
			self.get_logger().warn("CameraInfo が未受信です。")
			return

		rgb_image, depth_image = self.decode_rgbd(msg)
		if rgb_image is None or depth_image is None:
			self.get_logger().warn("RGBD のデコードに失敗しました。")
			return

		self.process_frame(rgb_image, depth_image, msg.header.frame_id)

	def rgb_depth_callback(self, rgb_msg: Image, depth_msg: Image) -> None:
		if self.model is None:
			return
		if self.camera_info is None:
			self.get_logger().warn("CameraInfo が未受信です。")
			return

		rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
		rgb_image = rgb_image[:, :, ::-1]  # BGR -> RGB
		depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")

		self.process_frame(rgb_image, depth_image, rgb_msg.header.frame_id)

	def decode_rgbd(self, msg: Image) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
		if msg.encoding != "32FC4":
			self.get_logger().warn(f"未対応のRGBDエンコーディング: {msg.encoding}")
			return None, None

		data = np.frombuffer(msg.data, dtype=np.float32)
		if data.size != msg.height * msg.width * 4:
			return None, None

		image = data.reshape((msg.height, msg.width, 4))
		rgb = image[:, :, :3]
		depth = image[:, :, 3]

		if rgb.max() <= 1.0:
			rgb = (rgb * 255.0).clip(0, 255).astype(np.uint8)
		else:
			rgb = rgb.astype(np.uint8)

		return rgb, depth

	def process_frame(self, rgb: np.ndarray, depth: np.ndarray, frame_id: str) -> None:
		results = self.model(rgb, verbose=False)[0]

		if results.boxes is None or len(results.boxes) == 0:
			return

		target_idx = self.find_target_box(results)
		if target_idx is None:
			return

		box = results.boxes[target_idx]
		x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().tolist()
		cx = int((x1 + x2) / 2.0)
		cy = int((y1 + y2) / 2.0)

		depth_value = self.get_depth_at(depth, cx, cy)
		if depth_value is None:
			self.get_logger().warn("有効な深度が取得できませんでした。")
			return

		x, y, z = self.project_to_3d(cx, cy, depth_value)
		if x is None:
			return

		point_msg = PointStamped()
		point_msg.header.stamp = self.get_clock().now().to_msg()
		point_msg.header.frame_id = frame_id
		point_msg.point.x = float(x)
		point_msg.point.y = float(y)
		point_msg.point.z = float(z)

		self.point_pub.publish(point_msg)

		class_name = results.names[int(box.cls[0].item())]
		conf = float(box.conf[0].item())
		self.get_logger().info(
			f"{class_name} ({conf:.2f}) -> 3D: ({x:.3f}, {y:.3f}, {z:.3f})"
		)

	def find_target_box(self, results) -> Optional[int]:
		best_idx = None
		best_conf = 0.0

		for i, box in enumerate(results.boxes):
			class_name = results.names[int(box.cls[0].item())]
			conf = float(box.conf[0].item())
			if class_name != self.target_class:
				continue
			if conf < self.conf_threshold:
				continue
			if conf > best_conf:
				best_conf = conf
				best_idx = i

		return best_idx

	def get_depth_at(self, depth: np.ndarray, x: int, y: int) -> Optional[float]:
		h, w = depth.shape[:2]
		if x < 0 or x >= w or y < 0 or y >= h:
			return None

		half = max(0, self.depth_window // 2)
		x0 = max(0, x - half)
		x1 = min(w, x + half + 1)
		y0 = max(0, y - half)
		y1 = min(h, y + half + 1)

		window = depth[y0:y1, x0:x1].astype(np.float32).reshape(-1)
		if window.size == 0:
			return None

		if depth.dtype == np.uint16:
			window = window * self.depth_unit_scale

		window = window[np.isfinite(window)]
		window = window[window > 0.0]
		if window.size == 0:
			return None

		return float(np.median(window))

	def project_to_3d(self, u: int, v: int, z: float) -> Tuple[Optional[float], Optional[float], Optional[float]]:
		if self.camera_info is None:
			return None, None, None

		k = self.camera_info.k
		fx = k[0]
		fy = k[4]
		cx = k[2]
		cy = k[5]

		if fx == 0.0 or fy == 0.0:
			return None, None, None

		x = (u - cx) * z / fx
		y = (v - cy) * z / fy
		return x, y, z


def main() -> None:
	rclpy.init()
	node = YoloRgbdRealsenseNode()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		node.destroy_node()
		rclpy.shutdown()


if __name__ == "__main__":
	main()
