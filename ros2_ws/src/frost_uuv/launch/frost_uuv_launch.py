import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='frost_uuv',
            executable='nav_instructions_pub'),
        # launch_ros.actions.Node(
        #     package='frost_uuv',
        #     executable='echo_data_pub'),
        # launch_ros.actions.Node(
        #     package='frost_uuv',
        #     executable='gps_data_pub'),
        launch_ros.actions.Node(
            package='frost_uuv',
            executable='leak_detected_sub'),
        launch_ros.actions.Node(
            package='frost_uuv',
            executable='voltage_sub'),
        launch_ros.actions.Node(
            package='frost_uuv',
            executable='humidity_sub'),
        launch_ros.actions.Node(
          package='seatrac',
          executable='modem_data_pub'),
  ])
