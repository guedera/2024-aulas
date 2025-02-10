import time
import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import cv2
import numpy as np
import matplotlib.pyplot as plt
from cv_bridge import CvBridge
from robcomp_util.module_aruco import Aruco3d

class ExplorandoOrdenado(Node):

    def __init__(self):
        super().__init__('explorador_node')
        self.aruco_detector = Aruco3d()

        self.bridge = CvBridge()
        self.cyellow = {
            'lower': (100, 60, 50),
            'upper': (255, 255, 255)
        }
        self.kernel = np.ones((10,10), np.uint8)

        self.subcomp = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

        time.sleep(1)

        self.timer = self.create_timer(0.1, self.control)

        self.robot_state = 'segue'
        self.state_machine = {
            'segue': self.segue,
            'stop': self.stop,
            'curva': self.curva
        }

        self.threshold = 5
        self.kp = 0.005

        self.aruconafrente = False
        self.distaruco = np.inf
        self.avistei = False

        # Inicialização de variáveis
        self.twist = Twist()
        self.cx = np.inf

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        h,w,_ = cv_image.shape
        self.w = w/2
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        _, aruco_results = self.aruco_detector.detectaAruco(cv_image)
        if len(aruco_results) > 0:
            for result in aruco_results:
                self.aruconafrente = True
                self.avistei = True
                self.distaruco = result['distancia']
        else:
            self.aruconafrente = False
        mask = cv2.inRange(hsv, self.cyellow['lower'], self.cyellow['upper'])
        mask[:int(h/2),:] = 0
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            contour = max(contours, key=cv2.contourArea)
            cv2.drawContours(cv_image, contour, -1, [255, 0, 0], 3)

            M = cv2.moments(contour)
            self.cx = int(M["m10"] / M["m00"])
            self.cy = int(M["m01"] / M["m00"])

            cv2.circle(cv_image, (self.cx, self.cy), 5, (0, 0, 255), -1)

            cv2.imshow("cv_image", mask)
            cv2.waitKey(1)
        else:
            return -1

    def calc_erro(self):
        self.erro = self.w - self.cx
        self.rot = self.erro * self.kp
        print('Erro Angular:', self.erro)
        
    def segue(self):
        if self.cx == np.inf:
            self.twist.angular.z = -0.4
        else:
            self.calc_erro()
            distancia = (self.distaruco * 0.05)-1
            print(distancia)
            print(self.avistei)
            if self.aruconafrente or self.avistei:
                if distancia > 0.3:
                    self.twist.linear.x = 0.3
                else:
                    self.twist.linear.x = distancia
                self.twist.angular.z = self.rot
            else:
                self.twist.linear.x = 0.3
                self.twist.angular.z = self.rot
        if distancia  <= 0.1:
            self.robot_state = 'curva'

    def stop(self):
        self.twist = Twist()

    def curva(self):
        print('Avistei o cruzamento e parei! Direita ou esquerda?')
        self.avistei = False

    def control(self):
        self.twist = Twist()
        print(f'Estado Atual: {self.robot_state}')
        self.state_machine[self.robot_state]()
        self.cmd_vel_pub.publish(self.twist)


def main(args=None):
    rclpy.init(args=args)
    ros_node = ExplorandoOrdenado()

    rclpy.spin(ros_node)

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()