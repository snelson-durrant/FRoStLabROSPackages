import rclpy
from rclpy.node import Node
from enum import Enum
from frost_interfaces.msg import PID, IMU, Depth, Echo, GPS
from frost_interfaces.srv import EmergencyStop, GetEcho, GetGPS

PID_PUB_TIMER_PERIOD = 1  # seconds
SERVICE_TIMEOUT = 1  # seconds
QOS_PROFILE = 10
ECHO_REQ = GetEcho.Request()
GPS_REQ = GetGPS.Request()


class States(Enum):
    RUN = 1
    STOP = 2


class Controller(Node):
    # Creates all of the publishers, subscriptions, services, and clients
    def __init__(self):
        super().__init__("controller")

        # Create the callback groups
        # main_callback_group - functions outside of the timer callback loop
        # aux_callback_group - functions inside of the timer callback loop
        self.main_callback_group = (
            rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
        )
        self.aux_callback_group = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()

        # Create the publishers
        self.nav_publisher = self.create_publisher(
            PID, "pid_request", QOS_PROFILE, callback_group=self.main_callback_group
        )
        self.echo_publisher = self.create_publisher(
            Echo, "echo_data", QOS_PROFILE, callback_group=self.aux_callback_group
        )
        self.gps_publisher = self.create_publisher(
            GPS, "gps_data", QOS_PROFILE, callback_group=self.aux_callback_group
        )
        self.timer = self.create_timer(
            PID_PUB_TIMER_PERIOD,
            self.timer_callback,
            callback_group=self.main_callback_group,
        )

        # Create the subscriptions
        # self.imu_subscription = self.create_subscription(
        #     IMU,
        #     "imu_data",
        #     self.imu_listener_callback,
        #     QOS_PROFILE,
        #     callback_group=self.main_callback_group,
        # )
        # self.imu_subscription  # prevent unused variable warning
        # self.depth_subscription = self.create_subscription(
        #     Depth,
        #     "depth_data",
        #     self.depth_listener_callback,
        #     QOS_PROFILE,
        #     callback_group=self.main_callback_group,
        # )
        # self.depth_subscription  # prevent unused variable warning

        # Create the services
        self.srv = self.create_service(
            EmergencyStop,
            "emergency_stop",
            self.emergency_stop_callback,
            callback_group=self.main_callback_group,
        )

        # Create the clients
        self.echo_cli = self.create_client(
            GetEcho, "echo_service", callback_group=self.aux_callback_group
        )
        while not self.echo_cli.wait_for_service(timeout_sec=SERVICE_TIMEOUT):
            self.get_logger().info("Echo service not available, waiting...")
        self.gps_cli = self.create_client(
            GetGPS, "gps_service", callback_group=self.aux_callback_group
        )
        while not self.gps_cli.wait_for_service(timeout_sec=SERVICE_TIMEOUT):
            self.get_logger().info("GPS service not available, waiting...")

        # Set initial variables
        self.state = States.RUN
        self.prev_velocity = 0.0
        self.prev_yaw = 0.0
        self.prev_pitch = 0.0
        self.prev_roll = 0.0
        self.prev_depth = 0.0
        self.stop = False
        self.counter = 0

    # Updates the recieved IMU data
    # def imu_listener_callback(self, msg):
    #     self.imu_accel_x = msg.accel_x
    #     self.imu_accel_y = msg.accel_y
    #     self.imu_accel_z = msg.accel_z
    #     self.imu_gyro_x = msg.gyro_x
    #     self.imu_gyro_y = msg.gyro_y
    #     self.imu_gyro_z = msg.gyro_z
    #     self.imu_mag_x = msg.mag_x
    #     self.imu_mag_y = msg.mag_y
    #     self.imu_mag_z = msg.mag_z
    #     self.imu_lin_accel_x = msg.lin_accel_x
    #     self.imu_lin_accel_y = msg.lin_accel_y
    #     self.imu_lin_accel_z = msg.lin_accel_z
    #     self.imu_grav_x = msg.grav_x
    #     self.imu_grav_y = msg.grav_y
    #     self.imu_grav_z = msg.grav_z
    #     self.imu_rot_vec_i = msg.rot_vec_i
    #     self.imu_rot_vec_j = msg.rot_vec_j
    #     self.imu_rot_vec_k = msg.rot_vec_k
    #     self.imu_geomag_rot_vec_i = msg.geomag_rot_vec_i
    #     self.imu_geomag_rot_vec_j = msg.geomag_rot_vec_j
    #     self.imu_geomag_rot_vec_k = msg.geomag_rot_vec_k
    #     self.imu_raw_accel_x = msg.raw_accel_x
    #     self.imu_raw_accel_y = msg.raw_accel_y
    #     self.imu_raw_accel_z = msg.raw_accel_z
    #     self.imu_raw_gyro_x = msg.raw_gyro_x
    #     self.imu_raw_gyro_y = msg.raw_gyro_y
    #     self.imu_raw_gyro_z = msg.raw_gyro_z
    #     self.imu_raw_mag_x = msg.raw_mag_x
    #     self.imu_raw_mag_y = msg.raw_mag_y
    #     self.imu_raw_mag_z = msg.raw_mag_z

    # Updates the recieved pressure sensor data
    # def depth_listener_callback(self, msg):
    #     self.depth_pressure = msg.pressure
    #     self.depth_depth = msg.depth
    #     self.depth_temperature = msg.temperature

    # Sets the state machine to STOP when EmergencyStop is requested
    def emergency_stop_callback(self, request, response):
        self.get_logger().info("EMERGENCY STOP EXECUTED")
        self.get_logger().info(request.error)
        self.state = States.STOP
        response.stopped = True
        return response

    # Gets the echo data from the echo service and publishes it
    def get_echo(self):
        echo_msg = Echo()

        self.echo_future = self.echo_cli.call_async(ECHO_REQ)
        while not self.echo_future.done():
            pass
        current_echo = self.echo_future.result()

        echo_msg.header = current_echo.header
        echo_msg.distance = current_echo.distance
        echo_msg.conf_level = current_echo.conf_level
        echo_msg.profile_data = current_echo.profile_data
        self.echo_publisher.publish(echo_msg)

        return echo_msg

    # Gets the gps data from the gps service and publishes it
    def get_gps(self):
        gps_msg = GPS()

        self.gps_future = self.gps_cli.call_async(GPS_REQ)
        while not self.gps_future.done():
            pass
        current_gps = self.gps_future.result()

        gps_msg.header = current_gps.header
        gps_msg.latitude = current_gps.latitude
        gps_msg.longitude = current_gps.longitude
        gps_msg.altitude = current_gps.altitude
        gps_msg.siv = current_gps.siv
        self.gps_publisher.publish(gps_msg)

        return gps_msg

    # Runs the state machine and high-level controller, publishes to pid_request
    def timer_callback(self):
        pid_msg = PID()

        if self.state == States.RUN:
            ###############################################
            # HIGH-LEVEL CONTROLLER CODE STARTS HERE
            ###############################################

            # echo_msg = self.get_echo()
            # gps_msg = self.get_gps()

            # TODO: Adjust this simple state machine
            # For a faster update time, adjust PID_PUB_TIMER_PERIOD
            if self.counter < 20:
                pid_msg.velocity = 0.0
                pid_msg.yaw = 90.0
                pid_msg.pitch = 0.0
                pid_msg.roll = 0.0
                pid_msg.depth = 0.0
                pid_msg.stop = False
            else:
                self.state = States.STOP

            self.counter += 1

            self.get_logger().info("PUBLISHING TO PID_REQUEST")

            ###############################################
            # HIGH-LEVEL CONTROLLER CODE ENDS HERE
            ###############################################

        elif self.state == States.STOP:
            pid_msg.stop = True

        # Only publish the new pid_request values if they change
        if (
            self.prev_velocity != pid_msg.velocity
            or self.prev_yaw != pid_msg.yaw
            or self.prev_pitch != pid_msg.pitch
            or self.prev_roll != pid_msg.roll
            or self.prev_depth != pid_msg.depth
            or self.stop != pid_msg.stop
        ):
            pid_msg.header.stamp = Node.get_clock(self).now().to_msg()
            self.get_logger().info(
                "Velocity (%d), Heading (%d, %d, %d), Depth (%d)"
                % (
                    pid_msg.velocity,
                    pid_msg.yaw,
                    pid_msg.pitch,
                    pid_msg.roll,
                    pid_msg.depth,
                )
            )
            self.nav_publisher.publish(pid_msg)

        self.prev_velocity = pid_msg.velocity
        self.prev_yaw = pid_msg.yaw
        self.prev_pitch = pid_msg.pitch
        self.prev_roll = pid_msg.roll
        self.prev_depth = pid_msg.depth
        self.stop = pid_msg.stop


def main(args=None):
    rclpy.init(args=args)

    executor = rclpy.executors.MultiThreadedExecutor()
    controller = Controller()
    executor.add_node(controller)

    executor.spin()

    executor.shutdown()
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
