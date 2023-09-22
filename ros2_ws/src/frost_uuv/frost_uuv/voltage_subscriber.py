import rclpy
from rclpy.node import Node
from frost_interfaces.srv import EmergencyStop
from frost_interfaces.msg import Volt

SERVICE_TIMEOUT = 1  # seconds
QOS_PROFILE = 10


class VoltageSubscriber(Node):
    def __init__(self):
        super().__init__("voltage_subscriber")
        self.subscription = self.create_subscription(
            Volt, "voltage", self.listener_callback, QOS_PROFILE
        )
        self.subscription  # prevent unused variable warning
        self.cli = self.create_client(EmergencyStop, "emergency_stop")
        while not self.cli.wait_for_service(timeout_sec=SERVICE_TIMEOUT):
            self.get_logger().info("EmergencyStop service not available, waiting...")
        self.req = EmergencyStop.Request()

    def send_request(self, err):
        self.req.error = err
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def listener_callback(self, msg):
        error = "ERROR: Low Voltage (" + msg.voltage + ")"
        self.send_request(error)


def main(args=None):
    rclpy.init(args=args)

    voltage_subscriber = VoltageSubscriber()

    rclpy.spin(voltage_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    voltage_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
