import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
from robcomp_util.odom import Odom
from robcomp_util.module_aruco import Aruco3d
import numpy as np
from robcomp_interfaces.msg import DetectionArray, Detection  # Import the DetectionArray message

class SeguidorNode(Node, Odom):

    def __init__(self):
        super().__init__('seguidor_node')
        Odom.__init__(self)
        self.aruco_detector = Aruco3d()

        self.bridge = CvBridge()
        self.timer = self.create_timer(0.20, self.control)

        self.state_machine = {
            'para': self.para,
            'segue': self.segue,
            'casa': self.casa,
            'gira250': self.gira250
        }

        self.twist = Twist()
        self.robot_state = 'segue'
        self.xtela = np.inf
        self.ytela = 0
        self.w = 0
        self.running = True
        self.kp = 0.007

        self.goal_points = [(1.12, 0.0), (0.0, 0.0)]
        self.current_goal_index = 0
        self.goal_tolerance = 0.2

        self.creepers = {
            'creeper_13': '',
            'creeper_21': '',
            'creeper_11': '',
            'creeper_25': '',
            'car': (0, 0),
            'dog': (0, 0),
            'cat': (0, 0)
        }

        self.vi250 = False
        self.x250_start = None
        self.y250_start = None
        self.moving_after_250 = False

        self.gira250_started = False
        self.yaw_previous = None
        self.yaw_accumulated = 0.0

        # Subscribers
        self.image_sub = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.image_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

        # Subscriber do mobilenet_detection
        self.mobilenet_sub = self.create_subscription(
            DetectionArray,
            'mobilenet_detection',
            self.mobilenet_callback,
            10)

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

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
            height, _, _ = cv_image.shape
            cv_image[:int(height/2), :] = 0

            amarelo_baixo = np.array([19, 75, 140])
            amarelo_alto = np.array([50, 245, 255])

            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            mask_amarelo = cv2.inRange(hsv_image, amarelo_baixo, amarelo_alto)
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))

            mask_amarelo = cv2.morphologyEx(mask_amarelo, cv2.MORPH_OPEN, kernel)
            mask_amarelo = cv2.morphologyEx(mask_amarelo, cv2.MORPH_CLOSE, kernel)

            contornos_amarelo, _ = cv2.findContours(mask_amarelo, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
            contornos_amarelo = sorted(contornos_amarelo, key=cv2.contourArea, reverse=True)

            _, aruco_results = self.aruco_detector.detectaAruco(imagem_cheia)
            for result in aruco_results:
                aruco_id = result['id'][0]
                creeper_key = f'creeper_{aruco_id}'

                if creeper_key in self.creepers:
                    if self.creepers[creeper_key] == '':
                        self.creepers[creeper_key] = 'creeper_place'
                        self.get_logger().info(f'{creeper_key} marcado como creeper_place')
                
                if aruco_id == 250:
                    self.vi250 = True #flag deteccaao aruco 250

                if aruco_id == 150 and self.all_creepers_found():
                    self.robot_state = 'casa'
                    self.get_logger().info('All creepers found and ArUco 150 detected, changing state to "casa"')

            creeper_place_count = list(self.creepers.values()).count('creeper_place')
            if creeper_place_count == 3 and 'labirinto' not in self.creepers.values():
                for key, value in self.creepers.items():
                    if value == '':
                        self.creepers[key] = 'labirinto'
                        self.get_logger().info(f'{key} marcado como labirinto')
                        break

            if len(contornos_amarelo) > 0:
                self.xtela, self.ytela = self.get_center(contornos_amarelo)[0]
                self.w = (cv_image.shape[1]) / 2
                cv2.drawContours(imagem_cheia, [contornos_amarelo[0]], -1, (0, 255, 0), 3)
                self.crosshair(imagem_cheia, (self.xtela, self.ytela), 5, (255, 0, 0))

            cv2.imshow('Image', imagem_cheia)
            cv2.waitKey(1)
        else:
            self.xtela = np.inf
            self.get_logger().info('Procurando faixas...')

    def para(self):
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist)
        self.get_logger().info('Robô parando')

    def segue(self):
        if self.vi250 and not self.moving_after_250:
            self.x250_start = self.x
            self.y250_start = self.y
            self.moving_after_250 = True
            self.get_logger().info('Aruco250 detectado, indo!')

        if self.moving_after_250:
            dx = self.x - self.x250_start
            dy = self.y - self.y250_start
            dist = np.hypot(dx, dy)
            self.get_logger().info(f'distancia pra 250: {dist:.2f} metros')

            if dist >= 1.15:
                self.robot_state = 'gira250'
                self.vi250 = False
                self.moving_after_250 = False
                self.x250_start = None
                self.y250_start = None
                self.get_logger().info('alinha aruco 250')

        self.twist.linear.x = 0.47
        if self.xtela == np.inf:
            self.twist.angular.z = 0.5
            self.twist.linear.x = 0.0
            self.get_logger().info('Procurando linha')

        else:
            erro = self.w - self.xtela
            rot = self.kp * erro
            self.twist.angular.z = rot

        print(self.creepers)
        
    def control(self):
        self.twist = Twist()
        self.state_machine[self.robot_state]()
        self.cmd_vel_pub.publish(self.twist)

    def crosshair(self, img, point, size, color):
        x, y = point
        cv2.line(img, (x - size, y), (x + size, y), color, 5)
        cv2.line(img, (x, y - size), (x, y + size), color, 5)
        return img

    def mobilenet_callback(self, msg):
        for detection in msg.deteccoes:
            if detection.classe in ['car', 'dog', 'cat']:
                if self.creepers[detection.classe] == (0, 0):
                    self.creepers[detection.classe] = (self.x, self.y)
                    self.get_logger().info(
                        f"{detection.classe} detected at position ({self.x}, {self.y})"
                    )

    def all_creepers_found(self):
        # Checka os creepers
        creeper_keys = ['creeper_11', 'creeper_21', 'creeper_11', 'creeper_25']
        all_creeper_places_found = all(self.creepers[key] in ['creeper_place', 'labirinto'] for key in creeper_keys)
        # Checka se bicycle, dog, cat achado!
        creeper_positions = [self.creepers[obj] for obj in ['car', 'dog', 'cat']]
        all_positions_found = all(pos != (0, 0) for pos in creeper_positions)
        return all_creeper_places_found and all_positions_found

    def casa(self):
        if self.current_goal_index < len(self.goal_points):
            goal_x, goal_y = self.goal_points[self.current_goal_index]
            dx = goal_x - self.x
            dy = goal_y - self.y
            distance = np.hypot(dx, dy)
            desired_angle = np.arctan2(dy, dx)
            angle_error = desired_angle - self.yaw

            angle_error = np.arctan2(np.sin(angle_error), np.cos(angle_error))
            print(self.creepers)

            if abs(angle_error) > 0.1:
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.5 * angle_error
            else:
                self.twist.linear.x = 0.5
                self.twist.angular.z = 0.1 * angle_error

            if distance < self.goal_tolerance:
                self.get_logger().info(f"Reached point {self.current_goal_index + 1}")
                self.current_goal_index += 1

                if self.current_goal_index == len(self.goal_points):
                    self.robot_state = 'para'
        else:
            
            self.robot_state = 'para'
        
    def gira250(self):
        if not self.gira250_started:
            
            self.gira250_started = True
            self.yaw_previous = self.yaw
            self.yaw_accumulated = 0.0
            self.get_logger().info('Começando o giro!')
        else:
            delta_yaw = self.yaw - self.yaw_previous
            delta_yaw = np.arctan2(np.sin(delta_yaw), np.cos(delta_yaw))
            self.yaw_accumulated += abs(delta_yaw)
            self.yaw_previous = self.yaw


            self.twist.angular.z = 0.25
            self.twist.linear.x = 0.0


            self.get_logger().info(f'graus atual: {np.degrees(self.yaw_accumulated):.2f} degrees')
            print(self.creepers)

            if self.yaw_accumulated >= 2 * np.pi:
                self.gira250_started = False
                self.yaw_previous = None
                self.yaw_accumulated = 0.0
                self.robot_state = 'segue'
                self.get_logger().info('Dei a volta, seguindo linha!')

def main(args=None):
    rclpy.init(args=args)
    ros_node = SeguidorNode()
    rclpy.spin(ros_node)
    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
