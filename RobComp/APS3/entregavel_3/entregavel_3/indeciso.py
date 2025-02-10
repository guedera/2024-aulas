import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from sensor_msgs.msg import LaserScan
from entregavel_3.laser import Laser
from geometry_msgs.msg import Twist
# Adicione aqui os imports necessários        


class IndecisoNode(Node,Laser): # Mude o nome da classe

    def __init__(self):
        super().__init__('indeciso_node')
        Laser.__init__(self)
        self.timer = self.create_timer(0.25, self.control)

        self.robot_state = 'forward'

        self.state_machine = {
            'forward': self.forward,
            'stop' : self.stop,
            'backward': self.backward
        }

        # Inicialização de variáveis
        self.twist = Twist()
        
        # Subscribers
        ## Coloque aqui os subscribers

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        ## Coloque aqui os publishers

    def stop(self):
        if (1.05 > min(self.front) and min(self.front) > 0.95):
            self.twist.linear.x = 0.0
            self.robot_state = 'stop'
        elif min(self.front) < 0.95:
            self.robot_state = 'backward'
        else:
            self.robot_state = 'forward'


    def backward(self):
        if min(self.front) < 0.95:
            self.twist.linear.x = -1.0
        elif min(self.front) > 1.05:
            self.robot_state = 'forward'
        else: 
            self.robot_state = 'stop'

    def forward(self):
        if min(self.front) > 1.05:
            self.twist.linear.x = 1.0
        elif min(self.front) < 0.95:
            self.robot_state = 'backward'
        else:
            self.robot_state = 'stop'

    def control(self):
        self.twist = Twist()
        print(f'Estado Atual: {self.robot_state}')
        self.state_machine[self.robot_state]()

        self.cmd_vel_pub.publish(self.twist)
        
            
def main(args=None):
    rclpy.init(args=args)
    ros_node = IndecisoNode() # Mude o nome da classe

    rclpy.spin(ros_node)

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()