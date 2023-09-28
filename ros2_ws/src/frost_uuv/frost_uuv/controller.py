import rclpy
from rclpy.node import Node
from enum import Enum
from frost_interfaces.msg import Nav, IMU, Depth, Echo, GPS
from frost_interfaces.srv import EmergencyStop, GetEcho, GetGPS

NAV_PUB_TIMER_PERIOD = 1  # seconds
SERVICE_TIMEOUT = 1  # seconds
QOS_PROFILE = 10
DEFAULT_SERVO = (90, 90, 90) # out of 180
DEFAULT_THRUSTER = 0 # out of 100


class States(Enum):
    RUN = 1
    STOP = 2


class Controller(Node):
    # Creates all of the publishers, subscriptions, services, and clients
    def __init__(self):
        super().__init__("controller")

        # Create the publishers
        self.nav_publisher = self.create_publisher(Nav, "nav_instructions", QOS_PROFILE)
        self.echo_publisher = self.create_publisher(Echo, "echo_data", QOS_PROFILE)
        self.gps_publisher = self.create_publisher(GPS, "GPS_data", QOS_PROFILE)
        self.timer = self.create_timer(NAV_PUB_TIMER_PERIOD, self.timer_callback)

        # Create the subscriptions
        self.imu_subscription = self.create_subscription(
            IMU, "imu_data", self.imu_listener_callback, QOS_PROFILE
        )
        self.imu_subscription  # prevent unused variable warning
        self.depth_subscription = self.create_subscription(
            Depth, "depth_data", self.depth_listener_callback, QOS_PROFILE
        )
        self.depth_subscription  # prevent unused variable warning

        # Create the services
        self.srv = self.create_service(
            EmergencyStop, "emergency_stop", self.emergency_stop_callback
        )

        # Create the clients
        self.echo_cli = self.create_client(GetEcho, "echo_service")
        while not self.echo_cli.wait_for_service(timeout_sec=SERVICE_TIMEOUT):
            self.get_logger().info("Echo service not available, waiting...")
        self.echo_req = GetEcho.Request()
        self.gps_cli = self.create_client(GetGPS, "gps_service")
        while not self.gps_cli.wait_for_service(timeout_sec=SERVICE_TIMEOUT):
            self.get_logger().info("GPS service not available, waiting...")
        self.gps_req = GetGPS.Request()

        # Set initial variables
        self.state = States.RUN
        self.prev_servo1, self.prev_servo2, self.prev_servo3 = DEFAULT_SERVO
        self.prev_thruster = DEFAULT_THRUSTER
        self.current_imu = 0
        self.current_depth = 0

    # Updates the recieved IMU data
    def imu_listener_callback(self, msg):
        # TODO: update variables here
        self.current_imu = 1

    # Updates the recieved GPS data
    def depth_listener_callback(self, msg):
        # TODO: update variables here
        self.current_depth = 1

    # Sets the state machine to STOP when EmergencyStop is requested
    def emergency_stop_callback(self, request, response):
        self.get_logger().info("EMERGENCY STOP EXECUTED")
        self.get_logger().info(request.error)
        self.state = States.STOP
        response.stopped = True
        return response

    # Runs the state machine and controller, publishes to nav_instructions
    def timer_callback(self):
        nav_msg = Nav()

        if self.state == States.RUN:
            ########################################
            # CONTROLLER CODE STARTS HERE
            ########################################

            # self.future = self.echo_cli.call_async(self.echo_req)
            # rclpy.spin_until_future_complete(self, self.future)
            # current_echo = self.future.result()
            # TODO: publish data

            # self.future = self.gps_cli.call_async(self.gps_req)
            # rclpy.spin_until_future_complete(self, self.future)
            # current_gps = self.future.result()
            # TODO: publish data

            nav_msg.servo1, nav_msg.servo2, nav_msg.servo3 = DEFAULT_SERVO
            nav_msg.thruster = DEFAULT_THRUSTER

            self.get_logger().info("PUBLISHING TO NAV_INSTRUCTIONS")

            ########################################
            # CONTROLLER CODE ENDS HERE
            ########################################

        elif self.state == States.STOP:
            nav_msg.servo1, nav_msg.servo2, nav_msg.servo3 = DEFAULT_SERVO
            nav_msg.thruster = DEFAULT_THRUSTER

        # Only publish the new nav_instructions values if they change
        if (
            self.prev_servo1 != nav_msg.servo1
            or self.prev_servo2 != nav_msg.servo2
            or self.prev_servo3 != nav_msg.servo3
            or self.prev_thruster != nav_msg.thruster
        ):
            nav_msg.header.stamp = Node.get_clock(self).now().to_msg()
            self.get_logger().info(
                "Servos (%d, %d, %d), Thruster (%d)"
                % (nav_msg.servo1, nav_msg.servo2, nav_msg.servo3, nav_msg.thruster)
            )
            self.nav_publisher.publish(nav_msg)

        self.prev_servo1 = nav_msg.servo1
        self.prev_servo2 = nav_msg.servo2
        self.prev_servo3 = nav_msg.servo3
        self.prev_thruster = nav_msg.thruster


def main(args=None):
    rclpy.init(args=args)

    controller = Controller()

    rclpy.spin(controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
