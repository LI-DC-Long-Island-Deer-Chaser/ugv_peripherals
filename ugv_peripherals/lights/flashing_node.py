import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

"""
Sends commands to serial_out to control the lights onboard the rover. 
Lights will be on for 5-6 
"""

opcode = "01"
led_positions = [
    "1",
    "2",
    "3",
    "4",
    "5",
    "6",
    "7",
    "8",
    "9",
    "A",
    "B",
    "C",
    "D",
    "E",   
]
white = "E" # F for white
dark = "F"
headlights = "0"
strips = "1"

class FlashingLightsNode(Node):
    def __init__(self):
        super().__init__('lights')
        
        self.flash_period_ = 0.2
        self.phase_period_ = 5
        
        self.delay_ = 0.035

        self.publisher_ = self.create_publisher(String, 'serial_out', 20)
        self.flash_timer_ = self.create_timer(self.flash_period_, self.flash_callback_)  
        self.phase_timer_ = self.create_timer(self.phase_period_, self.phase_callback_)  
        
        self.flashing = True
        self.flash_on = True
        self.cmd_ = String()

        self.off_ = True

    def flash_callback_(self):
        """Blink LED rapidly when in flashing mode."""
        if self.flashing:
            self.turn_on_all_LEDs() if self.flash_on else self.turn_off_all_LEDs()
            self.flash_on = not self.flash_on
            self.off_ = False
        else:
            # Ensure LEDs are off when not flashing
            if not self.off_:
                self.turn_off_all_LEDs()
                self.off_ = True

    def phase_callback_(self):
        self.flashing = not self.flashing
        state = 'FLASHING' if self.flashing else 'DARK'
        self.get_logger().info(f"Switching to {state} phase")

    def turn_on_all_LEDs(self):
        self.cmd_.data = opcode + headlights + white
        self.publisher_.publish(self.cmd_)
        time.sleep(self.delay_)
        self.cmd_.data = opcode + strips + white
        self.publisher_.publish(self.cmd_)
        time.sleep(self.delay_)

    def turn_off_all_LEDs(self):
        self.cmd_.data = opcode + headlights + dark
        self.publisher_.publish(self.cmd_)
        time.sleep(self.delay_)
        self.cmd_.data = opcode + strips + dark
        self.publisher_.publish(self.cmd_)
        time.sleep(self.delay_)


def main():
    rclpy.init()
    node = FlashingLightsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
