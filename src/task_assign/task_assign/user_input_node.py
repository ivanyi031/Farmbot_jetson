import rclpy
from rclpy.node import Node
from sent_message.srv import Message


class UserInputNode(Node):
    def __init__(self):
        super().__init__('user_input_node')
        self.client = self.create_client(Message, 'string_service')

        # Wait until the service is available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for the service to become available...')
        self.user_prompt()

        

    def user_prompt(self):
        user_input = input("Enter a prompt: ")

        # Call the service
        self.req = Message.Request()
        self.req.request = user_input
        self.future = self.client.call_async(self.req)
        self.future.add_done_callback(self.service_response_callback)

    def service_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Service response: "{response.response}"')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
        self.user_prompt()

def main(args=None):
    rclpy.init(args=args)
    node = UserInputNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()
