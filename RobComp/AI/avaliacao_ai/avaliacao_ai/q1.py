import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from geometry_msgs.msg import Twist
from robcomp_interfaces.msg import Conversation
from pacote_simulado.laser import Laser
from pacote_simulado.odom import Odom
import numpy as np
from math import sqrt


#nem rodar roda, com mais calma e mais paciencia dava
class Explorador(Node, Laser, Odom):
    
    def __init__(self):
        super().__init__('explorador_node') 
        Laser.__init__(self)
        Odom.__init__(self)


        self.timer = self.create_timer(0.25, self.control)
        self.state_machine = {
            'stop': self.stop,
            'wait_instruction': self.wait_instruction,
            'wait': self.wait,
            'aproxima': self.follow_walls
        }

        #Inicialização de variáveis
        self.twist = Twist()
        self.robot_state = 'wait_instruction'
        self.conversation = Conversation()
        self.conversation.message = 'Robo: Estou pronto para explorar'
        self.initial_pose = None
        self.dist_a = 0.5
        self.achou = False

        #Publicadores
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.handler_pub = self.create_publisher(Conversation, 'handler', 10)
        self.handler_pub.publish(self.conversation)

        #Assinantes
        self.handler_sub = self.create_subscription(Conversation, 'handler', self.handler_callback, 10)

    def handler_callback(self, msg):
        message = msg.message
        if message.startswith('Handler:'):
            instruction = message[len('Handler:'):].strip()
            if instruction == 'Aguarde mais um momento.':
                self.robot_state = 'wait'
            elif instruction == 'Vá por cima!' or instruction == 'Vá por baixo!':
                self.robot_state = 'aproxima'

    def wait_instruction(self):
        pass

    def wait(self):
        pass

    def aproxima(self):
        front = np.min(self.front)
        left = np.min(self.left)
        right = np.min(self.right)
        if self.achou == False:
            if front > 1.0 and left > 1.0 and right > 0:
                self.twist.linear.x = 0.5

        erro = self.goal_yaw - self.yaw
        erro = np.arctan2(np.sin(erro), np.cos(erro))

        if erro > np.deg2rad(2):
            #vira esquerda
            self.twist.angular.z = 0.35
        
        elif erro < np.deg2rad(-2):
            #vira a direita
            self.twist.angular.z = -0.35
        else:
            self.robot_state = 'stop'
            self.conversation.message = 'Robo: Cheguei'
            self.handler_pub.publish(self.conversation)
        

    def wall_following(self):
        front = np.min(self.front)
        left = np.min(self.left)
        right = np.min(self.right)
        
        if front < 1.0:
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.5
        elif left < 1.0:
            self.twist.linear.x = 0.2
            self.twist.angular.z = -0.1
        elif right < 1.0:
            self.twist.linear.x = 0.2
            self.twist.angular.z = 0.1
        else:
            self.twist.linear.x = 0.5
            self.twist.angular.z = 0.0

    def stop(self):
        self.twist = Twist()

    def control(self):
        self.twist = Twist()
        print(f'Estado Atual: {self.robot_state}')
        self.state_machine[self.robot_state]()
        self.cmd_vel_pub.publish(self.twist)

def main(args=None):
    rclpy.init(args=args)
    ros_node = Explorador()

    rclpy.spin(ros_node)

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
