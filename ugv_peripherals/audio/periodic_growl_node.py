#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import Trigger  # Replace with your own service type if needed

class PeriodicServiceCaller(Node):
    def __init__(self):
        super().__init__('periodic_growl_node')

        # Create a client for the /do_something service
        self.client = self.create_client(Trigger, '/audio/play_sound')

        # Wait until the service is available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Service not available, waiting again...')

        # Create a timer that calls the service every 10 seconds
        self.timer_period = 10.0  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback_)

    def timer_callback_(self):
        """Called every 10 seconds."""
        request = Trigger.Request()

        self.get_logger().info('Sending service request...')
        self.client.call_async(request)


def main(args=None):
    rclpy.init(args=args)
    node = PeriodicServiceCaller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
