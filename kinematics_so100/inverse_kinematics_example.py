#!/usr/bin/env python3
"""
Inverse kinematics solver for SO100 robot arm
Reads SO100.urdf to extract arm dimensions and computes inverse kinematics.
"""

import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Tuple, Optional


class SO100InverseKinematics:
    """Inverse kinematics solver for SO100 robot arm."""
    
    def __init__(self, urdf_path: str):
        """
        Initialize with URDF file path.
        
        Args:
            urdf_path: Path to SO100.urdf file
        """
        self.urdf_path = urdf_path
        self.link_positions = {}  # Store joint origins from URDF
        self.load_urdf()
        self.compute_link_lengths()
    
    def load_urdf(self):
        """Load and parse URDF file to extract joint information."""
        tree = ET.parse(self.urdf_path)
        root = tree.getroot()
        
        # Extract all joints and their origins
        for joint in root.findall('joint'):
            joint_name = joint.get('name')
            origin = joint.find('origin')
            if origin is not None:
                xyz = origin.get('xyz', '0 0 0').split()
                rpy = origin.get('rpy', '0 0 0').split()
                
                self.link_positions[joint_name] = {
                    'xyz': [float(x) for x in xyz],
                    'rpy': [float(x) for x in rpy]
                }
        
        print(f"Loaded URDF: {self.urdf_path}")
        print(f"Found {len(self.link_positions)} joints")
    
    def compute_link_lengths(self):
        """Compute link lengths from URDF joint origins."""
        # Extract distances between joints along the arm chain
        # Based on the URDF structure: base -> shoulder -> upper_arm -> lower_arm -> wrist
        
        self.l1 = abs(self.link_positions['shoulder_pan']['xyz'][2])  # Base to shoulder height
        
        # Shoulder to upper_arm (along y-axis)
        self.l2 = abs(self.link_positions['shoulder_lift']['xyz'][1])
        
        # Upper_arm to lower_arm (along y-axis)
        self.l3 = abs(self.link_positions['elbow_flex']['xyz'][1])
        
        # Lower_arm to wrist (along z-axis)
        self.l4 = abs(self.link_positions['wrist_flex']['xyz'][2])
        
        print(f"\nLink lengths extracted from URDF:")
        print(f"  l1 (base-shoulder): {self.l1:.4f} m")
        print(f"  l2 (shoulder-elbow): {self.l2:.4f} m")
        print(f"  l3 (elbow-wrist1): {self.l3:.4f} m")
        print(f"  l4 (wrist1-wrist2): {self.l4:.4f} m")
    
    def inverse_kinematics(self, target_x: float, target_y: float, target_z: float, 
                          target_yaw: float = 0.0) -> Optional[Tuple[float, float, float, float, float]]:
        """
        Compute inverse kinematics for target position.
        
        Assumes 4-DOF arm with:
        - Joint 1: Shoulder pan (rotation about Z)
        - Joint 2: Shoulder lift (pitch)
        - Joint 3: Elbow flex (pitch)
        - Joint 4: Wrist flex (pitch)
        - Joint 5: Wrist roll (yaw)
        
        Args:
            target_x: Target X position (m)
            target_y: Target Y position (m)
            target_z: Target Z position (m)
            target_yaw: Target yaw angle (rad)
        
        Returns:
            Tuple of 5 joint angles (rad) or None if no solution exists
        """
        # Joint 1: Shoulder pan - rotation about Z to point towards target
        theta1 = math.atan2(target_y, target_x)
        
        # Project XY onto horizontal plane for 2D IK calculation
        r = math.sqrt(target_x**2 + target_y**2)
        
        # Adjust for base height offset
        z_adjusted = target_z - self.l1
        
        # 2D IK for planar 3-link arm (l2, l3, l4)
        # Using law of cosines for elbow-up configuration
        
        # Distance from shoulder to target in the arm plane
        d = math.sqrt(r**2 + z_adjusted**2)
        
        # Check if target is reachable
        max_reach = self.l2 + self.l3 + self.l4
        if d > max_reach:
            print(f"Error: Target distance {d:.4f}m exceeds max reach {max_reach:.4f}m")
            return None
        
        if d < abs(self.l2 - self.l3 - self.l4):
            print(f"Error: Target too close to robot base")
            return None
        
        # Law of cosines for elbow angle
        cos_theta3 = (d**2 - self.l2**2 - self.l3**2 - self.l4**2 + 
                      2*self.l2*self.l3 + 2*self.l3*self.l4) / (2*(self.l2 + self.l3)*self.l4)
        
        # Simplified 3-link IK
        cos_theta3 = (d**2 - self.l2**2 - (self.l3 + self.l4)**2) / (2 * self.l2 * (self.l3 + self.l4))
        
        if abs(cos_theta3) > 1.0:
            # Try different configuration
            cos_theta3 = max(-1.0, min(1.0, cos_theta3))
        
        theta3 = math.acos(cos_theta3)
        
        # Angle from shoulder to target
        alpha = math.atan2(z_adjusted, r)
        
        # Angle formed by l3+l4 to horizontal
        beta = math.atan2(self.l3 * math.sin(theta3), self.l2 + self.l3 * math.cos(theta3))
        
        theta2 = alpha - beta
        
        # Wrist angle to compensate for arm orientation
        theta4 = -theta2 - theta3
        
        # Wrist roll (yaw)
        theta5 = target_yaw
        
        return (theta1, theta2, theta3, theta4, theta5)


def main():
    """Main function to demonstrate inverse kinematics."""
    # Find URDF file
    urdf_path = Path(__file__).parent.parent / "so100_description" / "so100.urdf"
    
    if not urdf_path.exists():
        print(f"Error: URDF file not found at {urdf_path}")
        sys.exit(1)
    
    # Initialize IK solver
    ik_solver = SO100InverseKinematics(str(urdf_path))
    
    # Example target position (x, y, z) in meters from base
    # and target yaw angle in radians
    target_x = 0.15  # 15 cm forward
    target_y = 0.0   # center
    target_z = 0.20  # 20 cm up
    target_yaw = 0.0  # no rotation
    
    print(f"\n--- Computing Inverse Kinematics ---")
    print(f"Target position: X={target_x:.3f}m, Y={target_y:.3f}m, Z={target_z:.3f}m")
    print(f"Target yaw: {target_yaw:.3f} rad")
    
    # Compute inverse kinematics
    result = ik_solver.inverse_kinematics(target_x, target_y, target_z, target_yaw)
    
    if result is not None:
        theta1, theta2, theta3, theta4, theta5 = result
        
        print(f"\n--- Joint Angles (Solution Found) ---")
        print(f"θ1 (Shoulder Pan): {math.degrees(theta1):7.2f}° ({theta1:7.4f} rad)")
        print(f"θ2 (Shoulder Lift): {math.degrees(theta2):7.2f}° ({theta2:7.4f} rad)")
        print(f"θ3 (Elbow Flex): {math.degrees(theta3):7.2f}° ({theta3:7.4f} rad)")
        print(f"θ4 (Wrist Flex): {math.degrees(theta4):7.2f}° ({theta4:7.4f} rad)")
        print(f"θ5 (Wrist Roll): {math.degrees(theta5):7.2f}° ({theta5:7.4f} rad)")
        
        print("\n✓ Inverse kinematics computed successfully")
    else:
        print("\n✗ No solution found for the target position")
        sys.exit(1)


if __name__ == "__main__":
    main()
