#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class UR3eJointStateReader(Node):
    def __init__(self):
        super().__init__('ur3e_joint_state_reader')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.listener_callback,
            10)
        self.joint_positions = None

    def listener_callback(self, msg: JointState):
        if not msg.name:
            return

        # UR3e has 6 joints; filter only them (you may adapt names if using a different URDF)
        ur_joints = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                     'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        
        joint_map = dict(zip(msg.name, msg.position))
        self.joint_positions = [joint_map.get(name, None) for name in ur_joints]

        assert all(pos is not None for pos in self.joint_positions), "Not all joint positions were found in the message"
            # self.get_logger().info(f'Current UR3e joint positions: {self.joint_positions}')
            
def get_joint_angles(args=None):
    rclpy.init(args=args)
    node = UR3eJointStateReader()
    for _ in range(5):
        rclpy.spin_once(node, timeout_sec=1.0)
    angles = node.joint_positions
    node.destroy_node()
    rclpy.shutdown()
    return angles

if __name__ == '__main__':
    print(get_joint_angles())
