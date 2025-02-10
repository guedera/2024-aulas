import rclpy
import numpy as np
import time
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from geometry_msgs.msg import Twist, Point
from robcomp_util.amcl import AMCL
# Adicione aqui os imports necessários


class Teseu(Node, AMCL):  # Mude o nome da classe, se necessário
    def __init__(self, points):
        Node.__init__(self, 'teseu_node')  # Mude o nome do nó, se necessário
        AMCL.__init__(self)  # Inicializa a classe AMCL
        time.sleep(1)

        # Inicialização de variáveis
        self.twist = Twist()
        self.threshold = np.pi / 180
        self.kp_linear = 0.8
        self.kp_angular = 0.5
        self.max_vel = 0.5
        self.points = points
        self.estagios = 0
        self.point = self.points[self.estagios]

        self.robot_state = 'center'
        self.state_machine = {
            'center': self.center,
            'goto': self.goto,
            'stop': self.stop,
            'controla_estagios': self.controla_estagios
        }

        self.timer = self.create_timer(0.25, self.control)

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def controla_estagios(self):
        self.point = self.points[self.estagios]
        self.robot_state = 'center'

    def get_angular_error(self):
        x = self.point.x - self.x
        y = self.point.y - self.y
        theta = np.arctan2(y, x)

        self.distance = np.sqrt(x**2 + y**2)  # Corrigido x*2 e y*2 para x**2 e y**2
        erro = theta - self.odom_yaw
        self.erro = np.arctan2(np.sin(erro), np.cos(erro))

        print('Erro: ', self.erro)
        self.twist.angular.z = self.erro * self.kp_angular

    def center(self):
        self.get_angular_error()

        if abs(self.erro) < np.deg2rad(4):
            self.robot_state = 'goto'

    def goto(self):
        self.get_angular_error()

        if self.distance > 0.10:
            linear_x = self.distance * self.kp_linear
            self.twist.linear.x = min(linear_x, self.max_vel)
        else:
            self.robot_state = 'stop'

    def stop(self):
        self.twist = Twist()
        self.estagios += 1
        self.robot_state = 'controla_estagios'

    def control(self):
        self.twist = Twist()
        print(f'Estado Atual: {self.robot_state}')
        self.state_machine[self.robot_state]()

        self.cmd_vel_pub.publish(self.twist)


def main(args=None):
    rclpy.init(args=args)
    points = [
        Point(x=1.2, y=-0.05, z=0.0),
        Point(x=1.2, y=0.85, z=0.0),
        Point(x=1.2, y=1.2, z=0.0),
        Point(x=0.6, y=1.2, z=0.0),
        Point(x=0.6, y=0.6, z=0.0)
    ]

    ros_node = Teseu(points)

    rclpy.spin(ros_node)

    ros_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':  # Corrigido '_main_' para '__main__'
    main()
