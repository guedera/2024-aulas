import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from geometry_msgs.msg import Twist
from entregavel_3.odom import Odom
import numpy as np
# Adicione aqui os imports necessários        


class QuadradoNode(Node, Odom): # Mude o nome da classe

    def __init__(self):
        super().__init__('quadrado_node')
        Odom.__init__(self)
        self.timer = self.create_timer(0.25, self.control)

        self.robot_state = 'walk'

        self.state_machine = {
            'walk': self.walk,
            'turn' : self.turn
        }

        # Inicialização de variáveis
        self.twist = Twist()
        current_time = self.get_clock().now().to_msg()
        current_time = float(current_time.sec) + float(current_time.nanosec)/10**9
        self.final_time = current_time + 2.9
        # Subscribers
        ## Coloque aqui os subscribers

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        ## Coloque aqui os publishers

    def walk(self):
        self.twist.linear.x = 0.2
        current_time = self.get_clock().now().to_msg()
        current_time = float(current_time.sec) + float(current_time.nanosec)/10**9
        if current_time > self.final_time + 2.9:
            self.goal_yaw = self.yaw + np.pi/2
            print("J")
            self.robot_state= 'turn'
        
        


    def turn(self):
        self.twist.linear.x = 0.0
        erro = self.goal_yaw - self.yaw
        erro = np.arctan2(np.sin(erro), np.cos(erro))
        current_time = self.get_clock().now().to_msg()
        current_time = float(current_time.sec) + float(current_time.nanosec)/10**9
        
        if erro > np.deg2rad(2):
            #vira esquerda
            self.twist.angular.z = 0.2
            print("E")
        elif erro < np.deg2rad(-2):
            #vira dir
            self.twist.angular.z = -0.2
            print("D")
        else:
            self.robot_state = 'walk'
            self.final_time = current_time + 2.9
            print("A")


    def control(self):
        self.twist = Twist()
        print(f'Estado Atual: {self.robot_state}')
        self.state_machine[self.robot_state]()
        self.cmd_vel_pub.publish(self.twist)
        
            
def main(args=None):
    rclpy.init(args=args)
    ros_node = QuadradoNode() # Mude o nome da classe

    rclpy.spin(ros_node)

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()