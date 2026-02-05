import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStatePublishExample(Node):
	def __init__(self) -> None:
		super().__init__("joint_state_publish_example")

		self.publisher_ = self.create_publisher(JointState, "joint_states", 10)

		self.id_list = [1, 2, 3, 4, 5, 6]
		self.angle_sequences = [
			[0.0, 0.0, 0.0, 0.0, -1.5, 0.0],
			[0.2, -0.2, 0.3, 0.0, -1.5, -1.5],
			[-0.4, 0.4, 0.0, -0.3, 0.0, -1.0],
			[0.5, 0.0, -0.4, 0.3, 0.2, 0.0],
			[0.0, -0.5, 0.4, 0.0, -0.2, 0.0],
		]
		self.sequence_index = 0

		self.timer = self.create_timer(6.0, self.publish_next)

	def publish_next(self) -> None:
		msg = JointState()
		msg.header.stamp = self.get_clock().now().to_msg()
		msg.name = [str(i) for i in self.id_list]
		msg.position = self.angle_sequences[self.sequence_index]

		self.publisher_.publish(msg)

		self.sequence_index = (self.sequence_index + 1) % len(self.angle_sequences)


def main() -> None:
	rclpy.init()
	node = JointStatePublishExample()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		node.destroy_node()
		rclpy.shutdown()


if __name__ == "__main__":
	main()
