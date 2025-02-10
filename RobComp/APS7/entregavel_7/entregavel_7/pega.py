import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from geometry_msgs.msg import Twist
from robcomp_util.laser import Laser
from garra import Garra
from robcomp_util.odom import Odom
from robcomp_interfaces.msg import DetectionArray

class PegaCreeper(Node, Laser, Garra, Odom):

    def __init__(self, cor, id):
        super().__init__('pega_creeper_node')
        Laser.__init__(self)
        Garra.__init__(self)
        Odom.__init__(self)

        self.cor_alvo = cor
        self.id_alvo = id
        self.twist = Twist()
        self.robot_state = 'search'
        self.kx, self.kz = 0.2, 0.05
        self.erro = 0
        self.flag_catchd, self.flag_getback = False, False

        self.state_machine = {
            'search': self.search,
            'follow': self.follow,
            'bring_home': self.bring_home,
            'stop': self.stop,
            'done': self.done
        }

        self.creeper_sub = self.create_subscription(
            DetectionArray,
            '/creeper',
            self.creeper_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.25, self.control)

    def search(self):
        if self.erro == 0:
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.15
        elif abs(self.erro) > 20:
            self.twist.angular.z = self.kz * self.erro / 100
        else:
            self.robot_state = 'follow'

    def follow(self):
        self.twist.linear.x = self.kx * min(self.front)
        if abs(self.erro) > 5:
            self.twist.angular.z = self.kz * self.erro / 100
        if min(self.front) <= 0.2:
            self.twist = Twist()
            self.robot_state = 'bring_home'
            self.goal_yaw = self.yaw + np.pi
            self.tempo_inicial = self.get_clock().now()

    def bring_home(self):
        if not self.flag_catchd:
            self.pegar_creeper()

        erro = self.goal_yaw - self.yaw
        erro = np.arctan2(np.sin(erro), np.cos(erro))
        self.twist.angular.z = 0.3 * erro

        if abs(erro) < np.deg2rad(10) and not self.flag_getback:
            self.twist.angular.z = 0.0
            self.tempo_inicial = self.get_clock().now()
            self.flag_getback = True

        if self.flag_getback and (self.get_clock().now().nanoseconds - self.tempo_inicial.nanoseconds) < 3 * 1e9:
            self.twist.linear.x = 0.2
        else:
            self.twist.linear.x = 0.0
            self.robot_state = 'stop'

    def pegar_creeper(self):
        self.flag_catchd = True
        self.controla_garra('open')
        self.controla_garra('mid')
        self.controla_garra('close')
        self.controla_garra('up')

    def stop(self):
        self.twist = Twist()
        self.controla_garra('mid')
        self.controla_garra('open')
        self.robot_state = 'done'

    def done(self):
        self.twist = Twist()

    def creeper_callback(self, array):
        for detection in array.deteccoes:
            classe = detection.classe.split('-')
            cor, id = classe[0], classe[1]
            if cor == self.cor_alvo and (id == self.id_alvo or id == f'[{self.id_alvo}]'):
                self.erro = detection.cx

    def control(self):
        self.twist = Twist()
        self.state_machine[self.robot_state]()
        self.cmd_vel_pub.publish(self.twist)

def main(args=None):
    rclpy.init(args=args)
    ros_node = PegaCreeper('green', 13)
    rclpy.spin(ros_node)
    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
