#!/usr/bin/env python3  # Tells the system this is a Python 3 executable script

import rclpy  # Imports rclpy, the ROS 2 Python client library
from rclpy.node import Node  # Imports the Node class, which is the base for all ROS 2 nodes
from std_msgs.msg import String
from collections import deque

from . import get_tty_acm

import serial  # Imports the pyserial module for serial communication

class SerialControllerNode(Node):  # Defines a new class for the ROS 2 node
    def __init__(self):  # Constructor for the node
        super().__init__('serial_controller_node')  # Initializes the node with the name 'serial_controller_node'
        self.publisher_ = self.create_publisher(String, 'serial_in', 10) # Initializes a publisher node with publishing to 'serial_in' topic to see who needs to listen IN on serial
        self.subscriber_ = self.create_subscription(String, 'serial_out', self.subscribe_and_send_serial, 10) # This node listens in on serial_out to prepare sending a message out to serial.

        get_serial_device = get_tty_acm.get_serial()
        self.get_logger().warning(f"get_serial_device is unstable, and should not be used. Result: {get_serial_device}")
        # Initializes a serial connection on /dev/ttyACM2 with 9600 baud rate
        self.ser = serial.Serial(f"/dev/{get_serial_device}", 9600, timeout=0.1)

        self.timer = self.create_timer(1.0, self.read_and_publish)  # 1.0 seconds
        self.busy_ = False
        self.to_be_sent_ = deque()


    def read_and_publish(self):  # Function that reads from and writes back to the serial port
        if self.ser.in_waiting > 0:    # Checks if there is any data waiting to be read
            msg_string = String()      # Make a String() Object
            raw = self.ser.readline()  # Reads one line of data from the serial port

            msg_string.data = hex(raw[0])[2:].rjust(2,'0')+hex(raw[1])[2:].rjust(2,'0')
            self.get_logger().info(f'Received: {raw}')  # Logs the received data
            #self.ser.write(raw)        # Writes the same data back to the serial port (echo)

            self.publisher_.publish(msg_string)
            byte_send_len = len(msg_string.data)
            self.get_logger().info(f'Publishing: {msg_string.data} : {byte_send_len} in \'serial_in\' topic.') # Publishing just for a feedback

    def subscribe_and_send_serial(self, msg):
        byte_send = bytes([int(msg.data[0:2], 16), int(msg.data[2:4], 16)])
        self.get_logger().info(f'Subscribed to serial_out, sending {byte_send}') # Printing to terminal for subscribe to the serial_out topic
        
        # If it's busy right now, toss it in the list.
        if self.busy_:
            self.get_logger().info("TOO BUSY TO SEND! Instead putting in list!")
            self.get_logger().info("busy list: %s"%(self.to_be_sent_))
            self.to_be_sent_.append(byte_send)
            return

        # Kinda like a mutex
        self.busy_ = True

        try:
            # prioritize the slow ones.
            if len(self.to_be_sent_) > 0:
                byte_send = self.to_be_sent_.popleft()

            # otherwise write what was originally intended
            bytes_written = self.ser.write(byte_send)
            self.ser.flush()
        
        except Exception as e:
            self.get_logger().error(f"Error: {e}")
        
        # Done writing to serial, no longer busy
        self.busy_ = False

def main(args=None):  # Entry point for the script
    rclpy.init(args=args)  # Initializes the ROS 2 Python communication
    node = SerialControllerNode()  # Creates an instance of the node
    rclpy.spin(node)  # Keeps the node running
    node.destroy_node()  # Destroys the node when shutting down
    rclpy.shutdown()  # Shuts down the ROS 2 system

if __name__ == '__main__':  # Runs the main function only if this script is run directly
    main()
