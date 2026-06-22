from ackermann_msgs.msg import AckermannDrive
import rclpy
from rclpy.node import Node
import serial as s
import math

port_serie = s.Serial(port='/dev/ttyACM0', baudrate=115200, bytesize=8, parity='N',
stopbits=1, timeout=None, write_timeout=None,
xonxoff=False, rtscts=False, dsrdtr=False)

class NoeudCommande(Node):
    def __init__(self):
        super().__init__('CoVAPSy_cmdR')
        self._vitesse_m_s = 0.0
        self._direction_degre = 0
        self.create_subscription(AckermannDrive, 'cmd_ackermann', self._cmd_ackermann_callback, 1)
        self.get_logger().info('noeud cree')

    def _cmd_ackermann_callback(self, message):
        self._vitesse_m_s = message.speed
        self._direction_degre = math.degrees(message.steering_angle) + 4
        if self._direction_degre > 25:
            self._direction_degre = 25
        elif self._direction_degre < -25:
            self._direction_degre = -25
        try:
            direction = int(float(90 + self._direction_degre))
        except:
            self.get_logger().warn("Bug direction : {}".format(self._direction_degre))
            direction = 90
        vitesse = int(4000 + self._vitesse_m_s * 1000) #4000 vitesse nulle
        port_serie.write(str.encode('v{:05d}d{:03d}\r'.format(vitesse, direction)))
        self.get_logger().info('v{:05d}d{:03d}'.format(vitesse, direction))

def main(args=None):
    rclpy.init(args=args)
    noeud = NoeudCommande()
    rclpy.spin(noeud)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
