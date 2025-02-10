import rclpy
import cv2
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy


from entregavel_6.module_net import MobileNetDetector

from robcomp_interfaces.msg import DetectionArray, Detection

class MobilenetNode(Node):
    def __init__(self):
        super().__init__('mobilenet_node')
        self.running = True

        # Subscribers
        ## Coloque aqui os subscribers
        self.bridge = CvBridge()
        self.detector = MobileNetDetector()

        # Subscriber imagem
        self.image_sub = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.image_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )

        # Subscriber mobilenet
        self.flag_sub = self.create_subscription(
            String,
            '/vision/mobilenet_flag',
            self.flag_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )

        # Publishers
        ## Coloque aqui os publishers
        self.detection_pub = self.create_publisher(
            DetectionArray,
            '/mobilenet_detection',
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

            _, detections = self.detector.detect(cv_image)

            deteccao = DetectionArray()
            deteccao.deteccoes = []

            for det in detections:
                detection = Detection()
                detection.classe = det['classe']

                (min_x, min_y, max_x, max_y) = det['bbox']

                cx = (min_x + max_x) / 2.0
                cy = (min_y + max_y) / 2.0

                detection.cx = float(cx)
                detection.cy = float(cy)

                deteccao.deteccoes.append(detection)
                print(f"Detecção: Classe={det['classe']}, BBox={det['bbox']}")
            print(f"Publicando {len(deteccao.deteccoes)} detecções")

            #publica
            self.detection_pub.publish(deteccao)

            cv2.imshow('Image', cv_image)
            cv2.waitKey(1)

        else:
            print('Image processing is paused')

def main(args=None):

    rclpy.init(args=args)

    mobilenet_node = MobilenetNode()

    rclpy.spin(mobilenet_node)

    mobilenet_node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
