#!/usr/bin/env python3
"""Sample joint state publisher for SO100 robot."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math


class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.1, self.publish_joint_states)
        self.time_counter = 0
        
    def publish_joint_states(self):
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # Joint names matching the URDF
        msg.name = [
            'shoulder_pan',
            'shoulder_lift',
            'elbow_flex',
            'wrist_flex',
            'wrist_roll',
            'gripper'
        ]
        
        # Example motion - smooth oscillation
        self.time_counter += 1
        t = self.time_counter * 0.01  # Time counter
        
        msg.position = [
            0.0,  # shoulder_pan
            math.sin(t) * 0.5,  # shoulder_lift - oscillate
            math.cos(t) * 1.0,  # elbow_flex - oscillate
            math.sin(t * 0.5) * 0.8,  # wrist_flex - slower oscillation
            0.0,  # wrist_roll
            0.5   # gripper
        ]
        
        msg.velocity = [0.0] * len(msg.name)
        msg.effort = [0.0] * len(msg.name)
        
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    joint_state_publisher = JointStatePublisher()
    
    try:
        rclpy.spin(joint_state_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        joint_state_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
