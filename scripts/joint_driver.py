import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
import time

class UR3eJointDriver(Node):
    def __init__(self, joint_angles=[1.57, 0.0, 0.0, -1.57, 0.0, 0.0], duration_sec=5):
        assert len(joint_angles) == 6, f"Target joint position should involve 6 angles; {len(joint_angles)} received"
        
        super().__init__('ur3e_joint_driver')
        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]

        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/scaled_joint_trajectory_controller/follow_joint_trajectory'
        )

        self.publisher = self.create_publisher(Float64MultiArray, 'ur3e_joint_angles', 10)
        # self.get_logger().info("UR3eDriver initialized. Waiting 2s before sending goal...")
        self.timer = self.create_timer(0.1, self.send_joint_angles) # This f**king timer is garbage collected whether a reference is kept or not
        self.goal = joint_angles
        self.duration_sec = duration_sec
        self.command_sent = False
        self.goal_done = False  # used to know when goal completes

    def set_goal(self, joint_angles, duration_sec):
        assert len(joint_angles) == 6, f"Target joint position should involve 6 angles; {len(joint_angles)} received"
        self.create_timer(0.1, self.send_joint_angles)
        self.goal = joint_angles
        self.duration_sec = duration_sec
        self.command_sent = False
        self.goal_done = False
        
    def send_joint_angles(self):
        if self.command_sent:
            return
        self.command_sent = True
        self.timer.cancel()

        msg = Float64MultiArray()
        msg.data = self.goal
        self.publisher.publish(msg)
        self.get_logger().info(f"Published joint angles: {self.goal}")

        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joint_names

        # home_point = JointTrajectoryPoint()
        # home_point.positions = [1.1868643760681152, -1.4915383395603676, 1.5335338751422327, -2.401619096795553, 4.643162727355957, -2.4154418150531214]
        # home_point.time_from_start.sec = int(self.duration_sec)
        # home_point.time_from_start.nanosec = int((self.duration_sec - int(self.duration_sec)) * 1e9)
        # trajectory_msg.points.append(home_point)

        point = JointTrajectoryPoint()
        point.positions = self.goal
        point.time_from_start.sec = int(self.duration_sec)
        point.time_from_start.nanosec = int((self.duration_sec - int(self.duration_sec)) * 1e9)
        trajectory_msg.points.append(point)

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory_msg

        # self.get_logger().info("Waiting for action server...")
        self._action_client.wait_for_server()

        # self.get_logger().info("Sending trajectory goal...")
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            self.goal_done = True
            return

        self.get_logger().info('Goal accepted. Waiting for result...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        # result = future.result().result
        status = future.result().status

        if status == 4:  # SUCCEEDED
            self.get_logger().info('Goal execution complete!')
        else:
            self.get_logger().warn(f'Goal failed with status code: {status}')

        self.goal_done = True

def move_joints(joint_angles, args=None):
    ''' 
    This function is a placeholder for the actual implementation that would send joint angles to the robot.
    The robot will be reset to initial pos if nothing's input
    '''
    assert not rclpy.ok()
    
    rclpy.init(args=args)
    node = UR3eJointDriver(joint_angles)
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    # Spin until goal is complete
    while rclpy.ok() and not node.goal_done:
        executor.spin_once(timeout_sec=0.1)

    node.destroy_node()
    rclpy.shutdown()
        
if __name__ == '__main__':
    move_joints()
    print("All done")
