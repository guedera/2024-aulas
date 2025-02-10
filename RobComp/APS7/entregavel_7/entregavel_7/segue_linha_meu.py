import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
from odom import Odom

import numpy as np

class SeguidorNode(Node, Odom):

    def __init__(self):
        super().__init__('seguidor_node')
        Odom.__init__(self)

        self.bridge = CvBridge()
        self.timer = self.create_timer(0.20, self.control)

        self.state_machine = {
            'para': self.para,
            'segue': self.segue
        }

        # Inicialização de variáveis
        self.twist = Twist()
        self.epsilon = 0.4  # Margem de erro da pos
        self.robot_state = 'segue'
        self.xtela = np.inf
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
        self.x = 0
        self.y = 0

        self.kp = 0.007

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
            imagem_cheia = cv_image.copy()
            height, _ , _ = cv_image.shape

            cv_image[:int(height/2),:] = 0

            # Processamento da imagem
            amarelo_baixo = np.array([19, 75, 140])
            amarelo_alto = np.array([50, 235, 255])

            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            mask_amarelo = cv2.inRange(hsv_image, amarelo_baixo, amarelo_alto)

            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))

            mask_amarelo = cv2.morphologyEx(mask_amarelo, cv2.MORPH_OPEN, kernel)
            mask_amarelo = cv2.morphologyEx(mask_amarelo, cv2.MORPH_CLOSE, kernel)

            # Encontra contornos
            contornos_amarelo, _ = cv2.findContours(mask_amarelo, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
            contornos_amarelo = sorted(contornos_amarelo, key=cv2.contourArea, reverse=True)

            def crosshair(img, point, size, color):
                x, y = point
                cv2.line(img, (x - size, y), (x + size, y), color, 5)
                cv2.line(img, (x, y - size), (x, y + size), color, 5)
                return img

            if len(contornos_amarelo) > 0:
                self.xtela, self.ytela = self.get_center(contornos_amarelo)[0]
                self.w = (cv_image.shape[1]) / 2 

                cv2.drawContours(imagem_cheia, [contornos_amarelo[0]], -1, (0, 255, 0), 3)
                imagem_cheia = crosshair(imagem_cheia, (self.xtela, self.ytela), 5, (255, 0, 0))

            cv2.imshow('Image', imagem_cheia)
            cv2.waitKey(1)
        else:
            self.xtela = np.inf
            self.get_logger().info('Procurando faixas...')

    def para(self):
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.get_logger().info('Robô parando')

    def segue(self):
        self.twist.linear.x = 0.5
        if self.definiu == False:
            self.xinit = self.x
            self.yinit = self.y
            self.definiu = True
            print('definiu!')

        print(abs(self.x - self.xinit))
        print(abs(self.y - self.yinit))
        if (abs(self.x - self.xinit) <= self.epsilon) and (abs(self.y - self.yinit) <= self.epsilon) and self.andoupouco:
            self.robot_state = 'para'
            return
        

        if self.contapasso >= 200 and self.andoupouco == False:
            self.andoupouco = True
            print('PitStop Permitido!')

        if self.xtela == np.inf:
            self.twist.angular.z = -0.5
            self.twist.linear.x = 0.0
        else:
            erro = self.w - self.xtela
            rot = self.kp * erro
            self.twist.angular.z = rot
            self.contapasso += 1

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
