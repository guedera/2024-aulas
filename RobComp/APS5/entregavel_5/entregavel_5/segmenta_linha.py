import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from rclpy.qos import ReliabilityPolicy, QoSProfile
from std_msgs.msg import String
import cv2
import numpy as np
from math import sqrt

class SegmentaLinhaNode(Node):

    def __init__(self):
        super().__init__('image_tool_node')
        self.running = True
        self.bridge = CvBridge()

        #Subscribers
        self.image_sub = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.image_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        self.flag_sub = self.create_subscription(
            String,
            '/vision/image_flag',
            self.flag_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )

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
        if not self.running:
            print('Processamento de imagem pausado')
            return

        def calcular_distancia(ponto1, ponto2):
            return sqrt((ponto2[0] - ponto1[0]) ** 2 + (ponto2[1] - ponto1[1]) ** 2)

        #Converter a imagem recebida
        cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")

        #Definir intervalo de cor amarela
        amarelo_baixo = np.array([20, 32, 32])
        amarelo_alto = np.array([35, 255, 255])
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask_amarelo = cv2.inRange(hsv_image, amarelo_baixo, amarelo_alto)

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(3,3))

        mask_amarelo = cv2.morphologyEx(mask_amarelo, cv2.MORPH_OPEN, kernel)

        # realiza a abertura
        mask_amarelo = cv2.morphologyEx(mask_amarelo, cv2.MORPH_CLOSE, kernel)

        #Encontrar contornos
        contornos_amarelo, _ = cv2.findContours(mask_amarelo, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

        if len(contornos_amarelo) > 0:

            self.x,self.y = self.get_center(contornos_amarelo)[0]
            self.w = (cv_image.shape[1]) / 2 #metade da larg

            amarelo_close = max(contornos_amarelo,key = cv2.contourArea)
            #Desenhar o contorno mais próximo
            cv2.drawContours(cv_image, [amarelo_close], -1, (0, 255, 0), 3)

        #Exibir a imagem
        cv2.imshow('Imagem', cv_image)
        cv2.waitKey(1)

#Função principal
def main(args=None):
    rclpy.init(args=args)
    ros_node = SegmentaLinhaNode()
    rclpy.spin(ros_node)
    ros_node.destroy_node()
    rclpy.shutdown()

#Inicializador
if __name__ == '__main__':
    main()
