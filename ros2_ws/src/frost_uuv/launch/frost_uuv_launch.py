import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            packaage='rosbag',
            executable='record',
            arguments=['-a -o ~/ros2_ws/rosbag/']),
        launch_ros.actions.Node(
            package='frost_uuv',
            executable='controller'),
        launch_ros.actions.Node(
            package='frost_uuv',
            executable='leak_detected_sub'),
        launch_ros.actions.Node(
            package='frost_uuv',
            executable='voltage_sub'),
        launch_ros.actions.Node(
            package='frost_uuv',
            executable='humidity_sub'),
        # launch_ros.actions.Node(
        #   package='seatrac',
        #   executable='modem_data_pub'),
  ])
