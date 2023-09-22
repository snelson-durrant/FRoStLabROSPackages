import rclpy
from rclpy.node import Node
from enum import Enum

from frost_interfaces.msg import Nav
from frost_interfaces.srv import EmergencyStop

import frost_uuv.nav_logic as nav_logic

NAV_PUB_TIMER_PERIOD = 1  # seconds
SERVO1_POS = 0
SERVO2_POS = 1
SERVO3_POS = 2
THRUSTER_POS = 3
AUV_STOPPED = [0, 0, 0, 0]


class States(Enum):
    RUN = 1
    STOP = 2


class NavInstructionsPublisher(Node):
    def __init__(self):
        super().__init__("nav_instructions_publisher")
        self.publisher_ = self.create_publisher(Nav, "nav_instructions")
        timer_period = NAV_PUB_TIMER_PERIOD
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.srv = self.create_service(
            EmergencyStop, "emergency_stop", self.emergency_stop_callback
        )
        self.state = States.RUN
        self.counter = True

    def emergency_stop_callback(self, request, response):
        self.get_logger().info("EMERGENCY STOP EXECUTED")
        self.get_logger().info(request.err)
        self.state = States.STOP
        response.stopped = True
        return response

    def timer_callback(self):
        msg = Nav()

        if self.state == States.RUN:
            # TODO: add controller code here
            array = nav_logic.navigate(self.counter)
            self.counter = not self.counter

        elif self.state == States.STOP:
            array = AUV_STOPPED

        msg.header.stamp = Node.get_clock(self).now().to_msg()
        msg.servo1 = array[SERVO1_POS]
        msg.servo2 = array[SERVO2_POS]
        msg.servo3 = array[SERVO3_POS]
        msg.thruster = array[THRUSTER_POS]
        self.get_logger().info(
            "Servos (%d, %d, %d), Thruster (%d)"
            % (msg.servo1, msg.servo2, msg.servo3, msg.thruster)
        )
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


if __name__ == "__main__":
    main()
