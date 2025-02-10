import rclpy
import cv2
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy


from entregavel_6.module_aruco import Aruco3d

from robcomp_interfaces.msg import DetectionArray, Detection

class aruco_node(Node):
    def __init__(self):
        super().__init__('aruco_node')
        self.running = True

        # Subscribers
        ## Coloque aqui os subscribers
        self.bridge = CvBridge()
        self.detector = Aruco3d()

        # Subscriber imagem
        self.image_sub = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.image_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )

        # Subscriber aruco
        self.flag_sub = self.create_subscription(
            String,
            '/vision/aruco_flag',
            self.flag_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )

        # Publishers
        ## Coloque aqui os publishers
        self.detection_pub = self.create_publisher(
            DetectionArray,
            '/aruco_detection',
            10
        )

    def flag_callback(self, msg):
        if msg.data.lower() == 'false':
            self.running = False
        else:
            self.running = True

    def image_callback(self, msg):
        if self.running:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8") # if Image

            # Faça aqui o processamento da imagem
            # ou chame uma classe ou função que faça isso
            _, detections = self.detector.detectaAruco(cv_image)

            deteccao = DetectionArray()
            deteccao.deteccoes = []
            
            for det in detections:
                detection = Detection()

                # Publica o ID do marcador como string
                detection.classe = str(det['id'][0])  # ids é um array

                # Calcula o centro do marcador
                cx, cy = det['centro']

                detection.cx = float(cx)
                detection.cy = float(cy)

                deteccao.deteccoes.append(detection)
                print(f"Detecção: ID={det['id'][0]}, Centro=({cx}, {cy})")
                cv_image = self.detector.drawAruco(cv_image, det)
            print(f"Publicando {len(deteccao.deteccoes)} detecções")

            #publica
            self.detection_pub.publish(deteccao)

            cv2.imshow('Image', cv_image)
            cv2.waitKey(1)

        else:
            print('Image processing is paused')

def main(args=None):

    rclpy.init(args=args)

    arucos_node = aruco_node()

    rclpy.spin(arucos_node)

    arucos_node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
