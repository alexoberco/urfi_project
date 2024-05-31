import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class OdomReplicator(Node):
    def __init__(self):
        super().__init__('odom_replicator')

        # Crear suscriptores para los tres tópicos de odometría
        self.odom_sub1 = self.create_subscription(
            Odometry,
            '/diff_controller_1/odom',
            self.odom_callback,
            10
        )
        self.odom_sub2 = self.create_subscription(
            Odometry,
            '/diff_controller_2/odom',
            self.odom_callback,
            10
        )
        self.odom_sub3 = self.create_subscription(
            Odometry,
            '/diff_controller_3/odom',
            self.odom_callback,
            10
        )

        # Crear publicador para el tópico combinado
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

    def odom_callback(self, msg):
        # Publicar el mensaje en el tópico /odom
        self.odom_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = OdomReplicator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
