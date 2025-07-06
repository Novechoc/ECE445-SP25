import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Trigger, SetBool
from serial import Serial
from stepper.device import Device
from stepper.stepper_core.parameters import DeviceParams
from stepper.stepper_core.configs import Address
import re
import time

class GripperController(Node):
    def __init__(self):
        super().__init__('gripper_controller')

        # Declare parameters
        self.declare_parameter('motor_port', '/dev/ttyAMA1')
        self.declare_parameter('motor_address', 1)
        port = self.get_parameter('motor_port').get_parameter_value().string_value
        address = self.get_parameter('motor_address').get_parameter_value().integer_value
        self.declare_parameter('min_pos', -50000)
        self.declare_parameter('max_pos', 850000)
        self.min_pos = self.get_parameter('min_pos').get_parameter_value().integer_value
        self.max_pos = self.get_parameter('max_pos').get_parameter_value().integer_value
        self.release_target_pos = 0

        # Connect to motor
        serial_conn = Serial(port, 115200, timeout=0.1)
        self.device = Device(DeviceParams(serial_connection=serial_conn, address=Address(address)))
        self.device.parse_cmd('ENA')

        # Sensor state and parameters
        self.fsr_triggered = False
        self.is_grasping = False
        self.is_releasing = False
        self.is_out_of_bounds = False

        # ROS interfaces
        self.fsr_subscriber = self.create_subscription(Bool, '/fsr_pressed', self.fsr_callback, 10)
        self.grasp_service = self.create_service(SetBool, '/grasp_object', self.handle_grasp)
        self.release_service = self.create_service(SetBool, '/release_object', self.handle_release)
        self.end_lock_service = self.create_service(Trigger, '/end_lock', self.handle_end_lock)
        self.grasp_release_check_timer = self.create_timer(0.1, self.check_grasp_release_status)
        self.get_logger().info('GripperController node has started.')

    def fsr_callback(self, msg):
        self.fsr_triggered = msg.data
        # self.get_logger().info(f'FSR state updated: {self.fsr_triggered}') # Uncomment for debugging

    def handle_grasp(self, request, response):
        if self.is_out_of_bounds:
            if not request.data:
                response.success = False
                response.message = "Gripper in Emergency Stop. Call /grasp_object with data=true to force."
                return response
            else:
                self.device.parse_cmd('ENA')
        self.get_logger().info('Grasping object...')
        self.fsr_triggered = False
        self.is_grasping = True
        self.device.parse_cmd('CLR') 
        self.device.parse_cmd('JOG 300')
        response.success = True
        response.message = 'Grasp command sent. Waiting for FSR trigger...'
        return response
    
    def handle_release(self, request, response):
        if self.is_out_of_bounds:
            if not request.data:
                response.success = False
                response.message = "Gripper in Emergency Stop. Call /release_object with data=true to force."
                return response
            else:
                self.device.parse_cmd('ENA')
        self.get_logger().info('Releasing object...')
        self.device.parse_cmd('ENA')
        self.device.parse_cmd('CLR') # Clear stall condition
        self.device.parse_cmd(f'MOV {self.release_target_pos}')
        self.is_releasing = True
        response.success = True
        response.message = 'Release command sent. Waiting for completion...'
        return response
    
    def handle_end_lock(self, request, response):
        self.get_logger().info('Removing out-of-bounds constraints...')
        self.device.parse_cmd('ENA')
        self.device.parse_cmd('CLR') # Clear stall condition
        self.is_out_of_bounds = False
        self.is_grasping = False
        self.is_releasing = False
        response.success = True
        response.message = 'Successfully cleared out-of-bounds constraints.'
        return response

    def check_grasp_release_status(self):
        current_pos = self.get_position()
        if self.is_grasping:
            if self.fsr_triggered:
                self.get_logger().info('Object detected by FSR — stopping motor.')
                self.device.parse_cmd('STP')
                self.is_grasping = False
            elif current_pos > self.max_pos:
                self.get_logger().warning(
                    f'Position {current_pos} out of bounds ({self.min_pos} - {self.max_pos}). Stopping motor.'
                )
                self.device.parse_cmd('STP')
                self.device.parse_cmd('DIS')
                self.is_grasping = False
                self.is_out_of_bounds = True
        elif self.is_releasing:
            if current_pos < self.min_pos:
                self.get_logger().warning(
                    f'Position {current_pos} out of bounds ({self.min_pos} - {self.max_pos}). Stopping motor.'
                )
                self.device.parse_cmd('STP')
                self.device.parse_cmd('DIS')
                self.is_releasing = False
                self.is_out_of_bounds = True
            elif abs(current_pos - self.release_target_pos) < 250:
                self.get_logger().info('Gripper fully opened.')
                self.is_releasing = False

    def get_position(self) -> int:
        s = str(self.device.parse_cmd('POS'))
        try:
            m = re.search(r'POS\s*=\s*(-?\d+)', s)
        except Exception as e:
            self.get_logger().error(f"Error parsing position: {e}")
            raise ValueError("Could not parse position from device response.")
        if not m:
            raise ValueError("Could not parse position from device response.")
        return int(m.group(1))
    
    def _cleanup(self):
        if rclpy.ok():  # Only log if rclpy is still active
            self.get_logger().info('Shutting down ......')
        try:
            self.device.parse_cmd('DIS')
        except Exception:
            pass

    def destroy_node(self):
        self._cleanup()
        super().destroy_node() # call the base-class teardown


def main(args=None):
    rclpy.init(args=args)
    node = GripperController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()