import rclpy
from rclpy.node import Node
from enum import Enum
from frost_interfaces.msg import Nav, IMU, Depth, Echo, GPS
from frost_interfaces.srv import EmergencyStop, GetEcho, GetGPS

NAV_PUB_TIMER_PERIOD = 1  # seconds
SERVICE_TIMEOUT = 1  # seconds
QOS_PROFILE = 10
DEFAULT_SERVO = (90, 90, 90)  # out of 180
DEFAULT_THRUSTER = 0  # out of 100


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
        self.gps_publisher = self.create_publisher(GPS, "gps_data", QOS_PROFILE)
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

    # Updates the recieved IMU data
    def imu_listener_callback(self, msg):
        self.imu_accel_x = msg.accel_x
        self.imu_accel_y = msg.accel_y
        self.imu_accel_z = msg.accel_z
        self.imu_gyro_x = msg.gyro_x
        self.imu_gyro_y = msg.gyro_y
        self.imu_gyro_z = msg.gyro_z
        self.imu_mag_x = msg.mag_x
        self.imu_mag_y = msg.mag_y
        self.imu_mag_z = msg.mag_z
        self.imu_lin_accel_x = msg.lin_accel_x
        self.imu_lin_accel_y = msg.lin_accel_y
        self.imu_lin_accel_z = msg.lin_accel_z
        self.imu_grav_x = msg.grav_x
        self.imu_grav_y = msg.grav_y
        self.imu_grav_z = msg.grav_z
        self.imu_rot_vec_i = msg.rot_vec_i
        self.imu_rot_vec_j = msg.rot_vec_j
        self.imu_rot_vec_k = msg.rot_vec_k
        self.imu_geomag_rot_vec_i = msg.geomag_rot_vec_i
        self.imu_geomag_rot_vec_j = msg.geomag_rot_vec_j
        self.imu_geomag_rot_vec_k = msg.geomag_rot_vec_k
        self.imu_raw_accel_x = msg.raw_accel_x
        self.imu_raw_accel_y = msg.raw_accel_y
        self.imu_raw_accel_z = msg.raw_accel_z
        self.imu_raw_gyro_x = msg.raw_gyro_x
        self.imu_raw_gyro_y = msg.raw_gyro_y
        self.imu_raw_gyro_z = msg.raw_gyro_z
        self.imu_raw_mag_x = msg.raw_mag_x
        self.imu_raw_mag_y = msg.raw_mag_y
        self.imu_raw_mag_z = msg.raw_mag_z

    # Updates the recieved pressure sensor data
    def depth_listener_callback(self, msg):
        self.depth_pressure = msg.pressure
        self.depth_depth = msg.depth
        self.depth_temperature = msg.temperature

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
        gps_msg = GPS()
        echo_msg = Echo()

        # Do we need this?
        self.gps_req = GetGPS.Request()
        self.echo_req = GetEcho.Request()
        self.gps_req.test = True
        self.echo_req.test = True

        if self.state == States.RUN:
            ########################################
            # CONTROLLER CODE STARTS HERE
            ########################################

            self.get_logger().info("STUCK HERE")
    
            self.future = self.echo_cli.call_async(self.echo_req)
            rclpy.spin_until_future_complete(self, self.future)
            current_echo = self.future.result()
            echo_msg.header = current_echo.header
            echo_msg.distance = current_echo.distance
            echo_msg.conf_level = current_echo.conf_level
            echo_msg.profile_data = current_echo.profile_data
            self.echo_publisher.publish(echo_msg)

            self.get_logger().info("TESTED ECHO SERVICE")

            self.future = self.gps_cli.call_async(self.gps_req)
            rclpy.spin_until_future_complete(self, self.future)
            current_gps = self.future.result()
            gps_msg.header = current_gps.header
            gps_msg.latitude = current_gps.latitude
            gps_msg.longitude = current_gps.longitude
            gps_msg.altitude = current_gps.altitude
            gps_msg.siv = current_gps.siv
            self.gps_publisher.publish(gps_msg)

            self.get_logger().info("TESTED GPS SERVICE")

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
