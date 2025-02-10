import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
from std_msgs.msg import String
import cv2
from odom import Odom

import numpy as np

class SeguidorNode(Node, Odom):

    def __init__(self):
        super().__init__('seguidor_node')
        Odom.__init__(self)

        self.bridge = CvBridge()
        self.timer = self.create_timer(0.25, self.control)

        self.state_machine = {
            'para': self.para,
            'centraliza': self.centraliza,
            'segue': self.segue
        }

        # Inicialização de variáveis
        self.twist = Twist()
        self.epsilon = 0.5  # Margem de erro
        self.robot_state = 'centraliza'
        self.xtela = 0
        self.ytela = 0
        self.w = 0
        self.running = True
        self.xinit = 0.0
        self.yinit = 0.0
        self.errox = 100
        self.erroy = 100
        self.andoupouco = False
        self.contapasso = 0
        self.definiu = False

        # Subscribers
        self.image_sub = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.image_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def flag_callback(self, msg):
        self.running = bool(msg.data)

    def get_center(self, contornos):
        centros = []
        for contorno in contornos:
            M = cv2.moments(contorno) 
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                centros.append((cX, cY))
        return centros

    def image_callback(self, msg):
        if self.running:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
            height, _ , _ = cv_image.shape
            cv_image[:int(height/2),:] = 0

            # Processamento da imagem
            amarelo_baixo = np.array([23, 50, 150])
            amarelo_alto = np.array([40, 230, 255])

            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            mask_amarelo = cv2.inRange(hsv_image, amarelo_baixo, amarelo_alto)

            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))

            mask_amarelo = cv2.morphologyEx(mask_amarelo, cv2.MORPH_OPEN, kernel)
            mask_amarelo = cv2.morphologyEx(mask_amarelo, cv2.MORPH_CLOSE, kernel)

            # Encontra contornos
            contornos_amarelo, _ = cv2.findContours(mask_amarelo, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

            def crosshair(img, point, size, color):
                x, y = point
                cv2.line(img, (x - size, y), (x + size, y), color, 5)
                cv2.line(img, (x, y - size), (x, y + size), color, 5)
                return img

            if len(contornos_amarelo) > 0:
                self.xtela, self.ytela = self.get_center(contornos_amarelo)[0]
                self.w = (cv_image.shape[1]) / 2  # Metade da largura

                cv2.drawContours(cv_image, [contornos_amarelo[0]], -1, (0, 255, 0), 3)
                cv_image = crosshair(cv_image, (self.xtela, self.ytela), 5, (255, 0, 0))

            cv2.imshow('Image', cv_image)
            cv2.waitKey(1)
        else:
            self.get_logger().info('Processamento pausado')

    def para(self):
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.get_logger().info('Robô parando')

    def centraliza(self):
        if abs(self.x - self.xinit) <= self.epsilon and abs(self.y - self.yinit) <= self.epsilon and self.andoupouco:
            self.robot_state = 'para'
            return
        erro = self.w - self.xtela
        self.twist.linear.x = 0.15
        if erro > 4:
            self.twist.angular.z = 0.4
            self.get_logger().info('Virando para a esquerda')
            self.contapasso += 1
        elif erro < -4:
            self.twist.angular.z = -0.4
            self.get_logger().info('Virando para a direita')
            self.contapasso += 1
        else:
            self.robot_state = 'segue'

    def segue(self):
        self.twist.linear.x = 0.2
        if self.definiu == False:
            self.xinit = self.x
            self.yinit = self.y
            self.definiu = True

        if (abs(self.x - self.xinit) <= self.epsilon) and (abs(self.y - self.yinit) <= self.epsilon) and self.andoupouco:
            self.robot_state = 'para'
            return

        if self.contapasso >= 120 and self.andoupouco == False:
            self.andoupouco = True
            print('japode!!')

        erro = self.w - self.xtela
        if erro > 4 or erro < -4:
            self.robot_state = 'centraliza'

        self.get_logger().info('Seguindo linha')

    def control(self):
        self.twist = Twist()
        self.state_machine[self.robot_state]()
        self.cmd_vel_pub.publish(self.twist)

def main(args=None):
    rclpy.init(args=args)
    ros_node = SeguidorNode()
    rclpy.spin(ros_node)
    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
