#!/usr/bin/env python

import sys
import moveit_commander
import rospy
from geometry_msgs.msg import Pose
from moveit_commander import RobotTrajectory
from moveit_commander import PlanningSceneInterface

def main():
    # Initialize MoveIt! commander and rospy
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('moveit_ur5_control', anonymous=True)
    
    # Initialize the move group for the UR3e
    robot = moveit_commander.RobotCommander()
    scene = PlanningSceneInterface()
    group_name = "manipulator"  # UR3e uses 'manipulator' as the default group name
    move_group = moveit_commander.MoveGroupCommander(group_name)
    
    # Set the reference frame for pose targets
    reference_frame = "base_link"
    move_group.set_pose_reference_frame(reference_frame)

    # Get the current state of the robot
    current_pose = move_group.get_current_pose().pose
    print(f"Current Pose: {current_pose}")
    
    end = False
    while not end:
        # Move the robot to a goal pose (example)
        goal_pose = Pose()
        coor = eval(f'({input("Please enter a 7D motion:")})')
        if len(coor) != 7:
            print("program end.")
            break
        goal_pose.position.x = coor[0]
        goal_pose.position.y = coor[1]
        goal_pose.position.z = coor[2]
        goal_pose.orientation.x = coor[3]
        goal_pose.orientation.y = coor[4]
        goal_pose.orientation.z = coor[5]
        goal_pose.orientation.w = coor[6]
        
        # Set the target pose
        move_group.set_pose_target(goal_pose)

        # Plan and execute the motion
        plan = move_group.plan()

        # Check if the movement was successful
        if plan:
            move_group.go(wait=True)
            print("Move succeeded!")
        else:
            print("Move failed!")

    # Shutdown MoveIt! commander
    moveit_commander.roscpp_shutdown()
    
    '''
    # Move in joint space
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = 1.0  # Modify the first joint's value
    move_group.go(joint_goal, wait=True)

    # Move in Catesian space
    waypoints = []
    current_pose = move_group.get_current_pose().pose
    # Define some poses in a list
    current_pose.position.z += 0.1
    waypoints.append(current_pose)
    (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
    move_group.execute(plan, wait=True)
    '''

if __name__ == "__main__":
    main()


