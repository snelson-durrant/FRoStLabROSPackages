import rclpy
from gps import *
from rclpy.node import Node

from frost_interfaces.msg import GPS

gpsd = gps(mode=WATCH_ENABLE|WATCH_NEWSTYLE)

class GPSDataPublisher(Node):

    def __init__(self):
        super().__init__('gpsdata_publisher')
        self.publisher_ = self.create_publisher(GPS, 'gps_data', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = GPS()
        nx = gpsd.next()
        # For a list of all supported classes and fields refer to:
        # https://gpsd.gitlab.io/gpsd/gpsd_json.html
        if nx['class'] == 'TPV':
            msg.latitude = getattr(nx,'lat', "Unknown")
            msg.longitude = getattr(nx,'lon', "Unknown")
        self.get_logger().info(
            'Latitude: %f, Longitude: %f' 
            % (msg.latitude, msg.longitude))
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    gps_data_publisher = GPSDataPublisher()

    rclpy.spin(gps_data_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    gps_data_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
