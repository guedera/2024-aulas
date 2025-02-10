import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np
from entregavel_3.laser import Laser

class LimpadorNode(Node, Laser):

    def __init__(self):
        super().__init__('limpador_node')
        Laser.__init__(self)
        self.timer = self.create_timer(0.25, self.control)

        self.robot_state = 'forward'

        self.state_machine = {
            'forward': self.forward,
            'turn': self.turn
        }

        
        self.twist = Twist()

        
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def custom_laser(self):
        
        self.lower_right = self.laser_msg[225 - self.openning:225 + self.openning]

    def forward(self):
        
        if np.min(self.front) > 0.5:
            self.twist.linear.x = 0.2
            print("trola")
        else:
            self.robot_state = 'turn'
            self.twist.linear.x = 0.0

    def turn(self):
        
        if np.min(self.lower_right) != np.min(self.laser_msg):
            self.twist.angular.z = 0.5
        else:
            self.robot_state = 'forward'
            self.twist.angular.z = 0.0

    def control(self):
        self.twist = Twist()
        print(f'Estado Atual: {self.robot_state}')
        self.state_machine[self.robot_state]()
        print(self.twist)
        self.cmd_vel_pub.publish(self.twist)


def main(args=None):
    rclpy.init(args=args)
    ros_node = LimpadorNode()

    rclpy.spin(ros_node)

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

