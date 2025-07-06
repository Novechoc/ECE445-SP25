import rclpy
# from rclpy.executors import SingleThreadedExecutor
from joint_driver import UR3eJointDriver
from joint_reader import UR3eJointStateReader
from kinematics import forward_kinematics, top_inverse_kinematics, side_inverse_kinematics
from kinematics import OFFSET, OFFSET_CAM, OFFSET_CAM_H, abs, sign, sign_off_mod, soff_mod_and_rad
from typing import Iterable
import time
import numpy as np

PI = 3.1415926

OFFSET_GRIPPER_ANGLE = 0.8 # in rad

# -180 to 180
def norm360_and_rad(angle):
    """Normalize angle to [-180, 180] degrees."""
    return ((angle + 180) % 360 - 180) / 180 * PI

def norm360(angle):
    """Normalize angle to [-180, 180] degrees."""
    return (angle + 180) % 360 - 180

# -90 to 90
def norm180_and_rad(angle):
    """Normalize angle to [-180, 180] degrees."""
    return ((angle + 90) % 180 - 90) / 180 * PI

def norm180(angle):
    """Normalize angle to [-180, 180] degrees."""
    return (angle + 90) % 180 - 90

# todo: unify rclpy init and shutdown in the main function
class UR3e:
    def __init__(self):
        rclpy.init()
        self.joint_reader = UR3eJointStateReader()
        self.joint_driver = UR3eJointDriver()
        self.pose = 'vertical'

    def __del__(self):
        self.joint_reader.destroy_node()
        self.joint_driver.destroy_node()
        rclpy.shutdown()

    def home(self, num=1, duration=5):
        '''move to the home position'''
        match num:
            case 0:
                self.move_to([0.3, 0, 0.2], duration=duration)
            case 1:
                self.move_to([0, -0.2, 0.2], duration=duration)
            case 2:
                self.move_to([-0.3, -0.05, 0.2], duration=duration)
    
    def gripper_angle(self): # in degree
        '''angle of the gripper in the world frame'''
        return norm360((self.joint_states()[-1] + PI/2 - self.joint_states()[0] - OFFSET_GRIPPER_ANGLE) / PI * 180)
    
    def yaw_angle(self): # in degree
        angles = self.joint_states()
        return (angles[0] - angles[4]) / PI * 180

    def joint_states(self):
        # return get_joint_angles()
        for _ in range(5):
            rclpy.spin_once(self.joint_reader, timeout_sec=1.0)
        return self.joint_reader.joint_positions
    
    def joint_act(self, joint_angles=[1.57, 0.0, 0.0, -1.57, 0.0, 0.0], duration_sec=3.0):
        self.joint_driver.set_goal(joint_angles, duration_sec)
        # executor = SingleThreadedExecutor()
        # executor.add_node(self.joint_driver)

        # Spin until goal is complete
        while rclpy.ok() and not self.joint_driver.goal_done:
            rclpy.spin_once(self.joint_driver, timeout_sec=0.1)

    def gripper_loc(self):
        joint_states = self.joint_states()
        loc = forward_kinematics(*joint_states)
        return loc
    
    def camera_loc(self):
        gripper_loc = self.gripper_loc()
        if self.pose == 'vertical':
            angle = np.arctan2(-gripper_loc[1], -gripper_loc[0]) % 180 + self.gripper_angle()
            gripper_loc[0] -= OFFSET_CAM_H * np.cos(angle)
            gripper_loc[1] -= OFFSET_CAM_H * np.sin(angle)
            gripper_loc[2] += OFFSET_CAM
            return [float(i) for i in gripper_loc]
    
    def move_to(self, loc: Iterable=None, yaw=0.0, gripper_angle=0.0, pose='vertical', duration=5.0):
        '''adjust angles with no input'''

        assert pose in ['vertical', 'horizontal'], "Invalid position. Choose from ['vertical', 'horizontal']"
        self.pose = pose
        self.yaw = sign_off_mod(yaw, 180)
        # self.gripper_angle = sign_off_mod(gripper_angle, 90)
        loc = loc if loc else self.gripper_loc()
        match pose:
            case 'vertical':
                goal_joint_angles = top_inverse_kinematics(*loc, self.yaw)
                # print(PI/2 - float(np.arctan2(-loc[1], -loc[0])))
                # print(mod_and_rad(gripper_angle + 20) - (PI/2 - float(np.arctan2(-loc[1], -loc[0]))))
                # goal_joint_angles.append(soff_mod_and_rad(gripper_angle + 20, 90) - (PI/2 - float(np.arctan2(-loc[1], -loc[0]))))
                if goal_joint_angles is None:
                    self.joint_driver.get_logger().info("Goal Invalid!")
                    return
                # goal_joint_angles.append(norm180_and_rad(gripper_angle) + OFFSET_GRIPPER_ANGLE - (PI/2-goal_joint_angles[0]))
                gripper_angle = (gripper_angle%180 - 180) / 180 * PI if gripper_angle%180 != 0 else 0
                # if gripper_angle > 0:
                #     gripper_angle = gripper_angle%180 - 180
                goal_joint_angles.append(gripper_angle + OFFSET_GRIPPER_ANGLE - (PI/2-goal_joint_angles[0]))
            case 'horizontal':
                goal_joint_angles = side_inverse_kinematics(*loc, self.yaw)
                if goal_joint_angles is None:
                    self.joint_driver.get_logger().info("Goal Invalid!")
                    return
                goal_joint_angles.append(soff_mod_and_rad(-135, 180))

        # infer the shortest paths
        adjusted_angles = [
            float(shortest_angle(curr, target))
            for curr, target in zip(self.joint_states(), goal_joint_angles)
        ]
        self.joint_act(adjusted_angles, duration)
        
    def move_by_diff(self, diff: Iterable=[0, 0, 0], diff_yaw=0, diff_gripper_angle=0, duration=5.0):
        current_location = self.gripper_loc()
        goal_location = [current_location[i] + diff[i] for i in range(len(diff))]
        self.move_to(goal_location, self.yaw_angle()+diff_yaw, self.gripper_angle()+diff_gripper_angle, self.pose, duration)
        
def shortest_angle(current, target):
    """Returns target adjusted to move in shortest direction from current."""
    return current + np.arctan2(np.sin(target - current), np.cos(target - current))