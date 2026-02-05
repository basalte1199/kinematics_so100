from sensor_msgs.msg import JointState
import rclpy
from rclpy.node import Node

ID_TO_JOINT = {
    '1': 'shoulder_pan',
    '2': 'shoulder_lift',
    '3': 'elbow_flex',
    '4': 'wrist_flex',
    '5': 'wrist_roll',
    '6': 'gripper',
}

class JointStateRemap(Node):
    def __init__(self):
        super().__init__('joint_state_remap')
        self.sub = self.create_subscription(
            JointState,
            '/joint_states_raw',
            self.cb,
            10
        )
        self.pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )

    def cb(self, msg):
        out = JointState()
        out.header = msg.header

        for name, pos in zip(msg.name, msg.position):
            if name in ID_TO_JOINT:
                out.name.append(ID_TO_JOINT[name])
                out.position.append(pos)

        self.pub.publish(out)

def main():
    rclpy.init()
    rclpy.spin(JointStateRemap())
