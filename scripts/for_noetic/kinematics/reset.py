import rospy
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Twist

def reset_robot():
    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        model_state = ModelState()

        model_state.model_name = "your_robot_model_name"  # Replace with your robot's model name
        
        # Reset position (set it to origin)
        model_state.pose = Pose()
        model_state.pose.position.x = 0.0
        model_state.pose.position.y = 0.0
        model_state.pose.position.z = 0.0
        model_state.pose.orientation.x = 0.0
        model_state.pose.orientation.y = 0.0
        model_state.pose.orientation.z = 0.0
        model_state.pose.orientation.w = 1.0

        # Reset velocity to zero
        model_state.twist = Twist()  # Zero velocities

        # Set the model state
        set_model_state(model_state)
        rospy.loginfo("Robot reset to zero state.")
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

if __name__ == "__main__":
    rospy.init_node('reset_robot_node')
    reset_robot()
