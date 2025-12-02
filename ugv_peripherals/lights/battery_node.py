import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

"""

The battery node will constantly monitor the /serial_in topic for the trig_cmd.
Once the trig_cmd is detected, the battery node will publish the battery status to /serial_out.

The format to /serial_out will be the first two most signficant nibbles (tag, op) followed by the battery percentage (2 digits) in hex.

The formula to calculate battery percentage is (x / cells_in_batt - dead_batt_voltage) / (full_batt_voltage - dead_batt_voltage),
where x is the battery voltage.


"""

opcode = "00"
led_position = "7" # top rear led
red = "6"
orange = "4"
green = "0"
red_light_cmd = opcode + led_position + red
orange_light_cmd = opcode + led_position + orange
green_light_cmd = opcode + led_position + green

cells_in_batt = 3
dead_batt_voltage = 3.3
critical_batt_voltage = 3.6
full_batt_voltage = 4.1

class MyCustomNode(Node):
    def __init__(self):
        super().__init__("battery_node")

        # needed to subscribe to /mavros/battery
        qos = QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1
        )

        # parameters for battery node
        self.latest_batt_voltage_ = None
        self.latest_batt_percent_ = None
        self.latest_batt_state_ = None
        # RED/ORANGE/GREEN
        self.batt_color_state = red_light_cmd

        # self.latest_batt_percent_ is updated every time new battery data comes in 1
        self.battery_subscription = self.create_subscription(BatteryState, '/mavros/battery', self.battery_callback, qos)

        # get battery_data OUT when requested for through serial_in
        # temporarily disabled. If you read this line of code,
        # please tell Pete to implement it, he's being lazy again.
        # self.ser_in_subscription = self.create_subscription(String, 'serial_in', self.battery_data_out_callback, 10)

        # This node publishes to serial out (along with many others)
        self.publisher_ = self.create_publisher(String, 'serial_out', 10)

        # update the battery status through LED through timer
        self.timer_period_ = 10 # seconds
        self.timer_ = self.create_timer(self.timer_period_, self.timer_callback_)

    def timer_callback_(self):
        # publish to serial_out
        result = String()
        result.data = self.batt_color_state
        self.get_logger().info("Now publishing!")
        self.publisher_.publish(result)

    def battery_callback(self, msg):
        if not msg.cell_voltage:
            self.get_logger().warn("Battery cell_voltage is empty, skipping callback")
            return
        self.latest_batt_voltage_ = msg.cell_voltage[0]
        try:
            self.latest_batt_percent_ = self.calculate_battery_percentage()
        except Exception as e:
            self.get_logger().error(f"Failed to calculate battery percentage: {e}")


    def battery_data_out_callback(self, msg):
        return
        # Someday I need to fix this code
        # It should be able to be triggered based off something.
        """
        # if the least significant byte of serial_in == trig_cmd then battery node will send out battery_percentage
        # publish to serial_out
        result = String()
        batt_percent_hex = str(hex(self.latest_batt_percent_))[2:]
        result.data = f"{trig_cmd[0]}{trig_cmd[1]}{batt_percent_hex}"
        if(self.latest_batt_percent_ == None):
            result.data = f"{trig_cmd[0]}{trig_cmd[1]}00"
        self.publisher_.publish(result)
        """

    def calculate_battery_percentage(self):
        result = (self.latest_batt_voltage_ / cells_in_batt) - dead_batt_voltage
        result = 100 * result / (full_batt_voltage - dead_batt_voltage)

        crit_lim = (critical_batt_voltage / cells_in_batt) - dead_batt_voltage
        crit_lim = 100 * crit_lim / (full_batt_voltage - dead_batt_voltage)

        if(result <= 0):
            self.latest_batt_state_ = f"SHUTTING DOWN! BATTERY AT {result:.2f}% DISCONNECT BATTERY ASAP."
            self.batt_color_state = red_light_cmd
        elif(result <= crit_lim):
            self.latest_batt_state_ = "CRITICAL! BATTERY AT {result:.2f}%. CHARGE BATTERY."
            self.batt_color_state = orange_light_cmd
        else:
            self.latest_batt_state_ = f"OKAY. BATTERY AT {result:.2f}%. CARRY ON."
            self.batt_color_state = green_light_cmd

        self.get_logger().info(self.latest_batt_state_)

        return int(result)


def main(args=None):
    rclpy.init()
    my_node = MyCustomNode()
    rclpy.spin(my_node)
    my_node.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()
