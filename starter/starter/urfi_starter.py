import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import tkinter as tk

class TeleopMultiController(Node):
    def __init__(self, master):
        super().__init__('teleop_multi_controller')
        self.master = master
        self.publisher1 = self.create_publisher(TwistStamped, '/diff_controller_1/cmd_vel', 10)
        self.publisher2 = self.create_publisher(TwistStamped, '/diff_controller_2/cmd_vel', 10)
        self.publisher3 = self.create_publisher(TwistStamped, '/diff_controller_3/cmd_vel', 10)

        self.linear_x = 0.0
        self.angular_z = 0.0

        # Setting up the key bindings
        self.master.bind('<KeyPress-w>', self.increase_x)
        self.master.bind('<KeyPress-x>', self.decrease_x)
        self.master.bind('<KeyPress-a>', self.increase_z)
        self.master.bind('<KeyPress-d>', self.decrease_z)
        self.master.bind('<KeyPress-s>', self.stop_motion)

    def publish_cmd_vel(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = self.linear_x
        msg.twist.angular.z = self.angular_z
        self.publisher1.publish(msg)
        self.publisher2.publish(msg)
        self.publisher3.publish(msg)

    def increase_x(self, event=None):
        if self.linear_x<2.0 and self.linear_x<0.2:
            self.linear_x += 0.3
        elif self.linear_x>0.2:
            self.linear_x += 0.1
        else:
            self.linear_x = 2.0
        self.publish_cmd_vel()

    def decrease_x(self, event=None):
        if self.linear_x > -2.0 and self.linear_x > -0.2:
            self.linear_x -= 0.3
        elif self.linear_x < -0.2:
            self.linear_x -= 0.1
        else:
            self.linear_x = -2.0
        self.publish_cmd_vel()

    def increase_z(self, event=None):
        if self.angular_z < 20.0:
            self.angular_z += 0.2
        else:
            self.angular_z = 20.0
        self.publish_cmd_vel()

    def decrease_z(self, event=None):
        if self.angular_z > -20.0:
            self.angular_z -= 0.2
        else:
            self.angular_z = -20.0
        self.publish_cmd_vel()

    def stop_motion(self, event=None):
        self.linear_x = 0.0
        self.angular_z = 0.0
        self.publish_cmd_vel()

def main(args=None):
    rclpy.init(args=args)
    root = tk.Tk()
    node = TeleopMultiController(root)
    root.mainloop()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
