import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from geometry_msgs.msg import Twist, Point
import numpy as np
import time
from robcomp_util.amcl import AMCL

class GoTo(Node, AMCL): 
    def __init__(self, points: list[Point] = [Point()]):
        Node.__init__(self, 'quadrado_node') 
        AMCL.__init__(self) 
        time.sleep(1)

        # Inicialização de variáveis
        self.twist = Twist()
        self.threshold = np.pi/180
        self.kp_linear = 0.3
        self.kp_angular = 0.3
        self.max_vel = 0.5
        self.points = points
        self.current_point_index = 0
        self.point = self.points[self.current_point_index]

        self.robot_state = 'center'
        self.state_machine = {
            'center': self.center,
            'goto': self.goto,
            'stop': self.stop
        }

        self.timer = self.create_timer(0.20, self.control)

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def get_angular_error(self):
        x = self.point.x - self.x
        y = self.point.y - self.y
        theta = np.arctan2(y , x)

        self.distance = np.sqrt(x**2 + y**2)
        erro = theta - self.odom_yaw
        self.erro = np.arctan2(np.sin(erro), np.cos(erro))

        print('Erro: ', self.erro)
        self.twist.angular.z = self.erro * self.kp_angular

    def center(self):
        self.get_angular_error()

        if abs(self.erro) < np.deg2rad(4):
            self.robot_state = 'goto'

    def goto(self):
        self.get_angular_error()

        if self.distance > 0.06:
            linear_x = self.distance * self.kp_linear
            self.twist.linear.x = min(linear_x, self.max_vel)
        else:
            #avanca ou stop
            self.current_point_index += 1
            if self.current_point_index < len(self.points):
                self.point = self.points[self.current_point_index]
                self.robot_state = 'center'
            else:
                self.robot_state = 'stop'
    
    def stop(self):
        self.twist = Twist()

    def control(self):
        self.twist = Twist()
        print(f'Estado Atual: {self.robot_state}')
        self.state_machine[self.robot_state]()

        self.cmd_vel_pub.publish(self.twist)
        
            
def main(args=None):
    rclpy.init(args=args)
    #pontos do mapa
    points = [Point(x=1.9, y=-0.11, z=0.0),
              Point(x=1.9, y=0.45, z=0.0),
              Point(x=1.2, y=1.10, z=0.0),
              Point(x=0.15, y=1.1, z=0.0), 
              Point(x=0.15, y=0.55, z=0.0),
              Point(x=0.69, y=0.5, z=0.0),
              Point(x=0.69, y=-0.05, z=0.0),
              Point(x=0.0, y=-0.049, z=0.0)
    ]
    ros_node = GoTo(points)

    rclpy.spin(ros_node)

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# Digite as coordenadas do ponto x,y do mapa, e.g 50,17: 114,62
# (2.0, -0.1499999999999999)
# Digite as coordenadas do ponto x,y do mapa, e.g 50,17: 116,74
# (2.1000000000000005, 0.4500000000000002)
# Digite as coordenadas do ponto x,y do mapa, e.g 50,17: 115,87
# (2.05, 1.1000000000000005)
# Digite as coordenadas do ponto x,y do mapa, e.g 50,17: 98,87
# (1.2000000000000002, 1.1000000000000005)
# Digite as coordenadas do ponto x,y do mapa, e.g 50,17: 79,87
# (0.25, 1.1000000000000005)
# Digite as coordenadas do ponto x,y do mapa, e.g 50,17: 79,76
# (0.25, 0.5500000000000003)
# Digite as coordenadas do ponto x,y do mapa, e.g 50,17: 90,75
# (0.7999999999999998, 0.5)
# Digite as coordenadas do ponto x,y do mapa, e.g 50,17: 90,64
# (0.7999999999999998, -0.04999999999999982)
# Digite as coordenadas do ponto x,y do mapa, e.g 50,17: 74,64
# (0.0, -0.04999999999999982)