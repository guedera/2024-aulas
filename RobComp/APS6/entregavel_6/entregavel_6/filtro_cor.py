import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
from std_msgs.msg import String
import cv2
from math import sqrt
import numpy as np
from module_aruco import Aruco3d
from module_net import MobileNetDetector
from geometry_msgs.msg import Point
# Adicione aqui os imports necessários

class filtro_cor_node(Node): # Mude o nome da classe

    def __init__(self):
        super().__init__('filtro_cor_node') # Mude o nome do nó

        # Inicialização de variáveis
        self.twist = Twist()
        self.bridge = CvBridge()

        self.running = True

        self.cor = 'azul'

        self.x = 0
        self.y = 0
        self.w = 0

        self.cores = {
            'verde': {
                'inferior': np.array([50, 150, 150]),
                'superior': np.array([70, 255, 255])
            },
            'vermelho': {
                'inferior': np.array([0, 150, 150]),
                'superior': np.array([10, 255, 255])
            },
            'vermelho2': {  
                'inferior': np.array([170, 150, 150]),
                'superior': np.array([179, 255, 255])
            },
            'azul': {
                'inferior': np.array([100, 150, 150]),
                'superior': np.array([130, 255, 255])
            }
        }

        # Subscribers
        ## Coloque aqui os subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )

        # Publishers
        ## Coloque aqui os publishers
        self.point_pub = self.create_publisher(Point, 'creeper_position', 10)

    def image_callback(self, msg):
        if self.running:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            imagem = cv_image.copy()
            hsv_image = cv2.cvtColor(imagem, cv2.COLOR_BGR2HSV)

            if self.cor in self.cores:
                color_limits = self.cores[self.cor]
                mask = cv2.inRange(hsv_image, color_limits['inferior'], color_limits['superior'])
                kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
                mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
                mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

                contornos, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
                if contornos:
                    x, y = self.get_center(contornos)[0]
                    closest_contour = max(contornos, key=cv2.contourArea)
                    cv2.drawContours(imagem, [closest_contour], -1, (200, 192, 255), 3)

                    point_msg = Point()
                    point_msg.x = float(x)
                    point_msg.y = float(y)
                    point_msg.z = float(cv_image.shape[1])
                    self.point_pub.publish(point_msg)
                else:
                    self.get_logger().info("Creeper não encontrado.")
            else:
                self.get_logger().warning(f"Cor '{self.cor}' não reconhecida.")

            cv2.imshow('Image', imagem)
            cv2.waitKey(1)
        else:
            self.get_logger().info('Processamento pausado')
        
    def get_center(self, contornos):
        centros = []
        for contorno in contornos:
            M = cv2.moments(contorno)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                centros.append((cX, cY))
        return centros
    
def main(args=None):
    rclpy.init(args=args)
    ros_node = filtro_cor_node() # Mude o nome da classe

    rclpy.spin(ros_node)

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

