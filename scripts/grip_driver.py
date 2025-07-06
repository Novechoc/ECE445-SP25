from std_srvs.srv import SetBool
import rclpy

def call_grasp_service(self):
    # Create the client
    client = self.create_client(SetBool, 'grasp_object')

    # Wait for the service to be available
    if not client.wait_for_service(timeout_sec=2.0):
        self.get_logger().error('grasp_object service not available!')
        return

    # Create the request, set data=True
    request = SetBool.Request()
    request.data = True

    # Call the service asynchronously
    future = client.call_async(request)
    rclpy.spin_until_future_complete(self, future)

    # Handle the response
    if future.result() is not None:
        result = future.result()
        self.get_logger().info(f'Service response: success={result.success}, message="{result.message}"')
    else:
        self.get_logger().error('Service call failed!')