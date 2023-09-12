import rclpy
from rclpy.node import Node

from frost_interfaces.msg import Nav
from frost_interfaces.srv import EmergencyStop

import frost_uuv.nav_logic as nav_logic


class NavInstructionsPublisher(Node):

    def __init__(self):
        super().__init__('nav_instructions_publisher')
        self.publisher_ = self.create_publisher(Nav, 'nav_instructions', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.srv = self.create_service(EmergencyStop, 'emergency_stop', self.emergency_stop_callback)
        self.stopped = False
        self.counter = True

    def emergency_stop_callback(self, request, response):
        self.get_logger().info('EMERGENCY STOP EXECUTED')
        response.stopped = True
        self.stopped = True
        return response

    def timer_callback(self):
        msg = Nav()

        if not self.stopped:
            # TODO: add fancy navigation function calls here
            array = nav_logic.navigate(self.counter)
            self.counter = not self.counter
        else:
            array = [0, 0, 0, 0]

        msg.header.stamp = Node.get_clock(self).now().to_msg()
        msg.servo1 = array[0]
        msg.servo2 = array[1]
        msg.servo3 = array[2]
        msg.thruster = array[3]
        self.get_logger().info(
            'Servos (%d, %d, %d), Thruster (%d)'
            % (msg.servo1, msg.servo2, msg.servo3, msg.thruster))
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    nav_instructions_publisher = NavInstructionsPublisher()

    rclpy.spin(nav_instructions_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    nav_instructions_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
