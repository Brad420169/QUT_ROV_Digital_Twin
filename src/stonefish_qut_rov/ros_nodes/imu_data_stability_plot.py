import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import math

class IMUPlotter(Node):
    def __init__(self):
        super().__init__('imu_plotter')
        self.subscription = self.create_subscription(
            Imu,
            '/qut_rov/imu',
            self.imu_callback,
            10)
        
        # Orientation (euler)
        self.roll  = deque(maxlen=200)
        self.pitch = deque(maxlen=200)
        self.yaw   = deque(maxlen=200)

        # Angular velocity
        self.ang_vel_x = deque(maxlen=200)
        self.ang_vel_y = deque(maxlen=200)
        self.ang_vel_z = deque(maxlen=200)

        # Linear acceleration
        self.lin_acc_x = deque(maxlen=200)
        self.lin_acc_y = deque(maxlen=200)
        self.lin_acc_z = deque(maxlen=200)

    def imu_callback(self, msg):
        # --- Orientation ---
        q = msg.orientation
        roll  = math.atan2(2*(q.w*q.x + q.y*q.z), 1 - 2*(q.x**2 + q.y**2))
        pitch = math.asin(2*(q.w*q.y - q.z*q.x))
        yaw   = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y**2 + q.z**2))
        self.roll.append(math.degrees(roll))
        self.pitch.append(math.degrees(pitch))
        self.yaw.append(math.degrees(yaw))

        # --- Angular velocity (rad/s) ---
        self.ang_vel_x.append(msg.angular_velocity.x)
        self.ang_vel_y.append(msg.angular_velocity.y)
        self.ang_vel_z.append(msg.angular_velocity.z)

        # --- Linear acceleration (m/s^2) ---
        self.lin_acc_x.append(msg.linear_acceleration.x)
        self.lin_acc_y.append(msg.linear_acceleration.y)
        self.lin_acc_z.append(msg.linear_acceleration.z)


def main():
    rclpy.init()
    node = IMUPlotter()

    # -------------------------------------------------------
    # Configure what you want to plot here
    # Each tuple is (data_deque, label, colour)
    # Just comment out lines you don't want
    # -------------------------------------------------------
    PLOTS = [
        # Orientation
        (node.roll,      'Roll (deg)',    'tab:blue'),
        (node.pitch,     'Pitch (deg)',   'tab:orange'),
        (node.yaw,       'Yaw (deg)',     'tab:green'),

        # Angular velocity
        # (node.ang_vel_x, 'AngVel X (rad/s)', 'tab:red'),
        # (node.ang_vel_y, 'AngVel Y (rad/s)', 'tab:purple'),
        # (node.ang_vel_z, 'AngVel Z (rad/s)', 'tab:brown'),

        # Linear acceleration
        # (node.lin_acc_x, 'LinAcc X (m/s²)', 'tab:pink'),
        # (node.lin_acc_y, 'LinAcc Y (m/s²)', 'tab:gray'),
        # (node.lin_acc_z, 'LinAcc Z (m/s²)', 'tab:cyan'),
    ]

    fig, ax = plt.subplots(figsize=(10, 5))

    def update(frame):
        rclpy.spin_once(node, timeout_sec=0.01)
        ax.clear()
        ax.set_ylabel('Value')
        ax.set_title('ROV IMU Data')
        ax.grid(True)
        for data, label, colour in PLOTS:
            ax.plot(list(data), label=label, color=colour)
        ax.legend(loc='upper right')

    ani = animation.FuncAnimation(fig, update, interval=50)
    plt.show()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()