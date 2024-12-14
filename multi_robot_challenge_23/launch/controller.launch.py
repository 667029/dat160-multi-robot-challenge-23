import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
         launch_ros.actions.Node(
            package='multi_robot_challenge_23',
            executable='robot_controller',
            namespace='tb3_0',
            name='robot_controller'),
        launch_ros.actions.Node(
            package='multi_robot_challenge_23',
            executable='position_publisher',
            namespace='tb3_0',
            name='position_publisher'),
        launch_ros.actions.Node(
            package='multi_robot_challenge_23',
            executable='marker_detection_node',
            namespace='tb3_0',
            name='marker_detection_node'),
        launch_ros.actions.Node(
            package='multi_robot_challenge_23',
            executable='robot_controller',
            namespace='tb3_1',
            name='robot_controller'),
        launch_ros.actions.Node(
            package='multi_robot_challenge_23',
            executable='position_publisher',
            namespace='tb3_1',
            name='position_publisher'),
        launch_ros.actions.Node(
            package='multi_robot_challenge_23',
            executable='marker_detection_node',
            namespace='tb3_1',
            name='marker_detection_node'),
        launch_ros.actions.Node(
            package='scoring',
            executable='scoring',
            name='scoring'),
        launch_ros.actions.Node(
            package='multi_robot_challenge_23',
            executable='coordination',
            name='coordination'),
    ])
