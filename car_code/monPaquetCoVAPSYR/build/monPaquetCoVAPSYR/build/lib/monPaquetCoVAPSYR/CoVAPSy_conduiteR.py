import rclpy
from ackermann_msgs.msg import AckermannDrive
from sensor_msgs.msg import LaserScan
from rclpy.node import Node

class Noeudconduite(Node):
    def __init__(self):
        super().__init__('CoVAPSy_conduiter')
        #ROS interface
        self.ackermann_publisher = self.create_publisher(AckermannDrive, 'cmd_ackermann', 1)
        self.create_subscription(LaserScan, 'scan', self._on_lidar_acquisition, 1)
        self.get_logger().info('noeud cree')

    def _on_lidar_acquisition(self, message):
        tableauLidar = list(message.ranges)
        self.get_logger().info(f'60 {tableauLidar[533]:.2f} et -60 {tableauLidar[2666]:.2f}')
        
        command_message = AckermannDrive()
        command_message.speed = 1.0
        
        try:
            command_message.steering_angle = 100 * (tableauLidar[479] - tableauLidar[238])
        except IndexError:
            command_message.steering_angle = 0.0
            
        if command_message.steering_angle > 18.0:
            command_message.steering_angle = 18.0
        if command_message.steering_angle < -18.0:
            command_message.steering_angle = -18.0
            
        self.ackermann_publisher.publish(command_message)
        self.get_logger().info(f'v={command_message.speed:.2f} m/s, d={command_message.steering_angle:.2f} rad')

def main(args=None):
    rclpy.init(args=args)
    noeud = Noeudconduite()
    rclpy.spin(noeud)
    rclpy.shutdown()

if __name__ == '__main__':
    main()