import rclpy
from rclpy.node import Node
from brping import Ping1D

from frost_interfaces.msg import Echo

class EchoDataPublisher(Node):

    def __init__(self):
        super().__init__('echodata_publisher')
        self.publisher_ = self.create_publisher(Echo, 'echo_data', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.myPing = Ping1D()
        self.myPing.connect_serial("/dev/echo_data", 115200)
        if self.myPing.initialize() is False:
            print("ERROR: Failed to initialize Ping!")
            exit(1)


    def timer_callback(self):

        data = self.myPing.get_distance()
        msg = Echo()
        msg.distance = data["distance"]
        msg.conf_level = data["confidence"]
        self.get_logger().info(
            'Distance: %d, Confidence Level: %d%%' 
            % (msg.distance, msg.conf_level))
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    echo_data_publisher = EchoDataPublisher()

    rclpy.spin(echo_data_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    echo_data_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
