#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import sys, termios, tty

class Teleop(Node):
    def __init__(self):
        super().__init__('teleop')
        self.pub = self.create_publisher(Float64MultiArray, '/updatedqutrov/setpoint/pwm', 10)

    def get_key(self):
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            return sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    def run(self):
        while True:
            key = self.get_key()
            print("KEY:", repr(key), flush=True)

            msg = Float64MultiArray()
            thrust = [0.0, 0.0, 0.0, 0.0]

            if key == 'i':
                print("Forward", flush=True)
                thrust = [1000, 1000, 1000, 1000]

            elif key == 'k':
                print("Backward", flush=True)
                thrust = [-1000, -1000, -1000, -1000]

            elif key == 'u':
                print("Up", flush=True)
                thrust = [1000, -1000, 0, 0]

            elif key == 'j':
                print("Down", flush=True)
                thrust = [-1000, 1000, 0, 0]

            elif key == 'x':
                print("Exit", flush=True)
                break

            msg.data = thrust
            self.pub.publish(msg)

def main():
    rclpy.init()
    node = Teleop()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()