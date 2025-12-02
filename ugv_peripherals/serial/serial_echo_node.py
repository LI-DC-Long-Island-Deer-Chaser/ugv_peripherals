#!/usr/bin/env python3  # Tells the system this is a Python 3 executable script

import rclpy  # Imports rclpy, the ROS 2 Python client library
from rclpy.node import Node  # Imports the Node class, which is the base for all ROS 2 nodes
from std_msgs.msg import String

class SerialEchoNode(Node):  # Defines a new class for the ROS 2 node
    def __init__(self):  # Constructor for the node
        super().__init__('serial_echo_node')  # Initializes the node with the name 'serial_controller_node'
        self.publisher_ = self.create_publisher(String, 'serial_out', 10) # Initializes a publisher node with publishing to 'serial_out' topic to (hopefully) get its message OUT!
        self.subscriber_ = self.create_subscription(String, 'serial_in', self.send_serial_out, 10) # This node listens in on serial_in to prepare sending a message out to serial_out.

        # Initializes a serial connection on /dev/ttyACM2 with 9600 baud rate
        # self.ser = serial.Serial('/dev/ttyACM2', 9600, timeout=0.1)

    def send_serial_out(self, msg):
        received = msg.data
        string_object = String()
        if received == "1234":
        # if received.startswith("ab") is also valid
        # if received[0:2] == SOMETHING!!!!!!

            self.get_logger().info("Received 1234. Sending 12FF")
            send_back = "12ff"
            string_object.data = send_back
            self.publisher_.publish(string_object) 
        else:
            self.get_logger().info(f"Did not receive 1234. Echoing back {received}")
            string_object.data = received
            self.publisher_.publish(string_object)
def main(args=None):  # Entry point for the script
    rclpy.init(args=args)  # Initializes the ROS 2 Python communication
    node = SerialEchoNode()  # Creates an instance of the node
    rclpy.spin(node)  # Keeps the node running
    node.destroy_node()  # Destroys the node when shutting down
    rclpy.shutdown()  # Shuts down the ROS 2 system

if __name__ == '__main__':  # Runs the main function only if this script is run directly
    main()
