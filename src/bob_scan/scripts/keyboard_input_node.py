#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pynput import keyboard


class KeyboardInputNode(Node):
    def __init__(self):
        super().__init__("keyboard_input_node")
        self.publisher_ = self.create_publisher(String, "keyboard_input", 10)
        self.listener = keyboard.Listener(on_press=self.on_key_press)
        self.listener.start()

    def on_key_press(self, key):
        msg = String()
        try:
            msg.data = f"Key {key.char} pressed"
        except AttributeError:
            msg.data = f"Special key {key} pressed"
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardInputNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
