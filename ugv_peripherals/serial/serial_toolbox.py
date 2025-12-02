import sys
import tty
import termios
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import curses

HEX_DIGITS = "0123456789abcdef"

class SerialToolbox(Node):
    def __init__(self):
        super().__init__('serial_toolbox')
        self.pub = self.create_publisher(String, 'serial_out', 10)
        self.sub = self.create_subscription(String, 'serial_in', self.serial_in_cb, 10)

        self.hex_values = [0] * 4
        self.cursor = 2
        self.has_data = False
        self.override_mode = False  # If true, skip waiting for serial input and allow all nibbles

        self.screen = None

    def serial_in_cb(self, msg):
        if self.override_mode:
            # Ignore incoming serial messages while in override mode
            return

        self.get_logger().info(f"Callback received data: {msg.data}")
        data = msg.data.strip().lower()
        if len(data) == 4 and all(c in HEX_DIGITS for c in data):
            self.hex_values = [HEX_DIGITS.index(c) for c in data]
            self.has_data = True
            self.cursor = 2
            self.draw_ui()
        else:
            self.get_logger().warn(f"Ignoring invalid serial_in: '{data}'")

    def draw_ui(self):
        self.screen.clear()

        # Header and data line
        if self.override_mode or self.has_data:
            hex_str = ''.join(HEX_DIGITS[v] for v in self.hex_values)
            self.screen.addstr(0, 0, hex_str)
            caret_line = [' '] * 4
            caret_line[self.cursor] = '^'
            self.screen.addstr(1, 0, ''.join(caret_line))

            if self.override_mode:
                self.screen.addstr(2, 0, "OVERRIDE MODE: editing all 4 nibbles")
            else:
                self.screen.addstr(2, 0, "Normal mode: editing last 2 nibbles")
        else:
            self.screen.addstr(0, 0, "Waiting for serial_in data...")

        # Controls
        self.screen.addstr(4, 0, "[W/S]=inc/dec | [A/D]=move | [Enter]=publish | [O]=toggle override | [Q]=quit")
        self.screen.refresh()

    def increment(self):
        if self.cursor_valid():
            self.hex_values[self.cursor] = (self.hex_values[self.cursor] + 1) % 16

    def decrement(self):
        if self.cursor_valid():
            self.hex_values[self.cursor] = (self.hex_values[self.cursor] - 1) % 16

    def move_left(self):
        if self.cursor > 0:
            self.cursor -= 1

    def move_right(self):
        if self.cursor < 3:
            self.cursor += 1

    def cursor_valid(self):
        # In normal mode, only last two nibbles editable; in override mode, all four
        return self.override_mode or self.cursor >= 2

    def publish_hex(self):
        if not (self.has_data or self.override_mode):
            self.get_logger().warn("No data received yet, can't publish.")
            return
        hex_str = ''.join(HEX_DIGITS[v] for v in self.hex_values)
        msg = String()
        msg.data = hex_str
        self.pub.publish(msg)
        self.get_logger().info(f'Published: {hex_str}')

    def toggle_override_mode(self):
        """Toggle override mode on/off."""
        self.override_mode = not self.override_mode
        if self.override_mode:
            self.has_data = True  # Skip waiting for serial data
            self.cursor = 0
            self.get_logger().info("Override mode ENABLED — editing all 4 nibbles.")
        else:
            self.has_data = False
            self.cursor = 2
            self.get_logger().info("Override mode DISABLED — waiting for serial_in again.")
        self.draw_ui()

    def run_curses(self, screen):
        self.screen = screen
        curses.cbreak()
        self.screen.keypad(True)
        self.screen.nodelay(True)  # make getch non-blocking

        self.draw_ui()

        try:
            while True:
                rclpy.spin_once(self, timeout_sec=0)
                c = self.screen.getch()
                if c == -1:
                    continue

                if c in (ord('q'), ord('Q')):
                    break

                elif c in (ord('o'), ord('O')):
                    self.toggle_override_mode()
                    continue

                elif not (self.has_data or self.override_mode):
                    self.draw_ui()
                    continue

                elif c in (ord('w'), ord('W'), curses.KEY_UP):
                    self.increment()
                elif c in (ord('s'), ord('S'), curses.KEY_DOWN):
                    self.decrement()
                elif c in (ord('a'), ord('A'), curses.KEY_LEFT):
                    self.move_left()
                elif c in (ord('d'), ord('D'), curses.KEY_RIGHT):
                    self.move_right()
                elif c in (ord('f'), ord(' '), 10, 13):  # F, Space, or Enter
                    self.publish_hex()
                    if not self.override_mode:
                        self.has_data = False
                    self.draw_ui()
                    continue

                self.draw_ui()

        finally:
            curses.nocbreak()
            self.screen.keypad(False)
            curses.echo()
            curses.endwin()


def main(args=None):
    rclpy.init(args=args)
    node = SerialToolbox()
    try:
        curses.wrapper(node.run_curses)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
