import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from math import sqrt
import numpy as np
from module_aruco import Aruco3d
from geometry_msgs.msg import Point

class AproximaNode(Node):  # Mudança no nome da classe para maiúsculas
    
    def __init__(self):
        super().__init__('base_control_node')  # Nome do nó mantido
        self.timer = self.create_timer(0.25, self.control)
        self.bridge = CvBridge()

        self.robot_state = 'stop'
        self.state_machine = {
            'segue': self.segue,
            'centraliza': self.centraliza,
            'stop': self.stop
        }

        # Inicialização de variáveis
        self.twist = Twist()
        self.robot_state = 'centraliza'
        self.x = 0
        self.y = 0
        self.running = True
        self.w = 0
        self.passeia = False
        
        # Subscribers
        self.creeper_sub = self.create_subscription(
            Point,
            'creeper_position',
            self.creeper_position_callback,  # Corrigido o callback
            10)

        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

    # Callback para o tópico creeper_position
    def creeper_position_callback(self, msg):
        """Recebe a posição do creeper e atualiza self.x e self.y."""
        self.x = msg.x
        self.get_logger().info(f'Creeper position received: x={self.x}, y={self.y}')
    
    def image_callback(self, msg):
        if self.running:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            imagem = cv_image.copy()
            Arucos = Aruco3d()

            _, results = Arucos.detectaAruco(imagem)

            if results:

                self.w = (imagem.shape[1]) / 2
                self.y = results[0]['distancia']

            else:
                self.w = (imagem.shape[1]) / 2
                self.passeia = True
                self.y = -1
                self.x = -1

            cv2.imshow('Image', imagem)
            cv2.waitKey(1)

        else:
            self.get_logger().info('Processamento pausado')

    def stop(self):
        self.twist = Twist()
        if self.passeia:
            self.passeia = False
            self.robot_state = 'centraliza'
        if self.y > 50:
            self.robot_state = 'segue'

    def segue(self):
        self.twist = Twist()
        if self.y > 50:
            self.twist.linear.x = 0.3
            print(self.y)
        else:
            self.robot_state = 'stop'

    def centraliza(self):
        self.twist = Twist()
        erro = self.w - self.x
        if erro > 4:
            self.twist.angular.z = 0.2
            self.get_logger().info('Virando para a esquerda')
        elif erro < -4:
            self.twist.angular.z = -0.2
            self.get_logger().info('Virando para a direita')
        else:
            self.robot_state = 'segue'

    def control(self):
        self.twist = Twist()
        print(f'Estado Atual: {self.robot_state}')
        self.state_machine[self.robot_state]()
        self.cmd_vel_pub.publish(self.twist)

def main(args=None):
    rclpy.init(args=args)
    ros_node = AproximaNode()  # Nome da classe atualizado

    rclpy.spin(ros_node)

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
