import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from gpiozero import DigitalInputDevice

class FSRPublisher(Node):
    def __init__(self):
        super().__init__('fsr_publisher')

        # Declare a parameter 'gpio_pin' with default value 17
        self.declare_parameter('gpio_pin', 17)
        gpio_pin = self.get_parameter('gpio_pin').get_parameter_value().integer_value

        # Initialize GPIO pin 17 for FSR input with debounce
        self.fsr = DigitalInputDevice(gpio_pin, bounce_time=0.1)

        # Publisher for FSR status (True = pressed, False = not pressed)
        self.publisher_ = self.create_publisher(Bool, 'fsr_pressed', 10)

        # Internal state to track current pressure status
        self.is_pressed = not self.fsr.value

        # Callbacks to update the internal state
        self.fsr.when_activated = self._on_released
        self.fsr.when_deactivated = self._on_pressed

        # Timer to publish the current status at 10 Hz
        self.timer = self.create_timer(0.1, self._publish_status)

        self.get_logger().info('FSR sensor node started.')

    def _on_pressed(self):
        self.is_pressed = True
        self.get_logger().info('Pressure detected (pressed)')

    def _on_released(self):
        self.is_pressed = False
        self.get_logger().info('Pressure released')

    def _publish_status(self):
        msg = Bool()
        msg.data = self.is_pressed
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = FSRPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()