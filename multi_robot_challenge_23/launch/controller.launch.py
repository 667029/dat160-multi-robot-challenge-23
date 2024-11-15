import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='multi_robot_challenge_23',
            executable='report_marker',
            namespace='tb3_0',
            name='report_marker'),
        launch_ros.actions.Node(
            package='multi_robot_challenge_23',
            executable='report_marker',
            namespace='tb3_1',
            name='report_marker'),
        launch_ros.actions.Node(
            package='multi_robot_challenge_23',
            executable='braitenberg',
            namespace='tb3_0',
            name='braitenberg'),
        launch_ros.actions.Node(
            package='multi_robot_challenge_23',
            executable='braitenberg',
            namespace='tb3_1',
            name='braitenberg'),
        launch_ros.actions.Node(
            package='scoring',
            executable='scoring',
            name='scoring'),
        # launch_ros.actions.Node(
        #     package='multi_robot_challenge_23',
        #     executable='bug2',
        #     namespace='tb3_0',
        #     name='bug2'),
        # launch_ros.actions.Node(
        #     package='multi_robot_challenge_23',
        #     executable='goToPoint',
        #     namespace='tb3_0',
        #     name='goToPoint'),
        # launch_ros.actions.Node(
        #     package='multi_robot_challenge_23',
        #     executable='wallFollower',
        #     namespace='tb3_0',
        #     name='wallFollower'),
    ])
