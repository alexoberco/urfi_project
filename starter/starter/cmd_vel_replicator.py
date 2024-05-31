import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class CmdVelReplicator(Node):
    def __init__(self):
        super().__init__('cmd_vel_replicator')

        # Crear el suscriptor al nuevo tópico
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.subscription  # Evita advertencias sobre variables no utilizadas

        # Crear los publicadores para los otros tres tópicos
        self.publisher1 = self.create_publisher(TwistStamped, '/diff_controller_1/cmd_vel', 10)
        self.publisher2 = self.create_publisher(TwistStamped, '/diff_controller_2/cmd_vel', 10)
        self.publisher3 = self.create_publisher(TwistStamped, '/diff_controller_3/cmd_vel', 10)

    def cmd_vel_callback(self, msg):
        # Crear un mensaje TwistStamped
        stamped_msg = TwistStamped()
        stamped_msg.header.stamp = self.get_clock().now().to_msg()
        stamped_msg.twist = msg

        # Publicar el mensaje en los tres tópicos
        self.publisher1.publish(stamped_msg)
        self.publisher2.publish(stamped_msg)
        self.publisher3.publish(stamped_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelReplicator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
