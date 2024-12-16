import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
         launch_ros.actions.Node(
            package='multi_robot_challenge_23',
            executable='robot_controller',
            namespace='tb3_0',
            name='robot_controller_tb3_0'),
        launch_ros.actions.Node(
            package='multi_robot_challenge_23',
            executable='robot_controller',
            namespace='tb3_1',
            name='robot_controller_tb3_1'),
        launch_ros.actions.Node(
            package='multi_robot_challenge_23',
            executable='robot_controller',
            namespace='tb3_2',
            name='robot_controller_tb3_2'),
        launch_ros.actions.Node(
            package='multi_robot_challenge_23',
            executable='robot_controller',
            namespace='tb3_3',
            name='robot_controller_tb3_3'),
        launch_ros.actions.Node(
            package='multi_robot_challenge_23',
            executable='robot_controller',
            namespace='tb3_4',
            name='robot_controller_tb3_4'),
        # launch_ros.actions.Node(
        #     package='multi_robot_challenge_23',
        #     executable='robot_controller',
        #     namespace='tb3_5',
        #     name='robot_controller'),
        # launch_ros.actions.Node(
        #     package='multi_robot_challenge_23',
        #     executable='robot_controller',
        #     namespace='tb3_6',
        #     name='robot_controller'),
        # launch_ros.actions.Node(
        #     package='multi_robot_challenge_23',
        #     executable='robot_controller',
        #     namespace='tb3_7',
        #     name='robot_controller'),
        # launch_ros.actions.Node(
        #     package='multi_robot_challenge_23',
        #     executable='robot_controller',
        #     namespace='tb3_8',
        #     name='robot_controller'),
         launch_ros.actions.Node(
            package='multi_robot_challenge_23',
            executable='position_publisher',
            namespace='tb3_0',
            name='position_publisher_tb3_0'),
        launch_ros.actions.Node(
            package='multi_robot_challenge_23',
            executable='position_publisher',
            namespace='tb3_1',
            name='position_publisher_tb3_1'),
        launch_ros.actions.Node(
            package='multi_robot_challenge_23',
            executable='position_publisher',
            namespace='tb3_2',
            name='position_publisher_tb3_2'),
        launch_ros.actions.Node(
            package='multi_robot_challenge_23',
            executable='position_publisher',
            namespace='tb3_3',
            name='position_publisher_tb3_3'),
        launch_ros.actions.Node(
            package='multi_robot_challenge_23',
            executable='position_publisher',
            namespace='tb3_4',
            name='position_publisher_tb3_4'),
        # launch_ros.actions.Node(
        #     package='multi_robot_challenge_23',
        #     executable='position_publisher',
        #     namespace='tb3_5',
        #     name='position_publisher'),
        # launch_ros.actions.Node(
        #     package='multi_robot_challenge_23',
        #     executable='position_publisher',
        #     namespace='tb3_6',
        #     name='position_publisher'),
        # launch_ros.actions.Node(
        #     package='multi_robot_challenge_23',
        #     executable='position_publisher',
        #     namespace='tb3_7',
        #     name='position_publisher'),
        # launch_ros.actions.Node(
        #     package='multi_robot_challenge_23',
        #     executable='position_publisher',
        #     namespace='tb3_8',
        #     name='position_publisher'),
        launch_ros.actions.Node(
            package='multi_robot_challenge_23',
            executable='marker_detection_node',
            namespace='tb3_0',
            name='marker_detection_node_tb3_0'),
        launch_ros.actions.Node(
            package='multi_robot_challenge_23',
            executable='marker_detection_node',
            namespace='tb3_1',
            name='marker_detection_node_tb3_1'),
        launch_ros.actions.Node(
            package='multi_robot_challenge_23',
            executable='marker_detection_node',
            namespace='tb3_2',
            name='marker_detection_node_tb3_2'),
        launch_ros.actions.Node(
            package='multi_robot_challenge_23',
            executable='marker_detection_node',
            namespace='tb3_3',
            name='marker_detection_node_tb3_3'),
        launch_ros.actions.Node(
            package='multi_robot_challenge_23',
            executable='marker_detection_node',
            namespace='tb3_4',
            name='marker_detection_node_tb3_4'),
        # launch_ros.actions.Node(
        #     package='multi_robot_challenge_23',
        #     executable='marker_detection_node',
        #     namespace='tb3_5',
        #     name='marker_detection_node'),
        # launch_ros.actions.Node(
        #     package='multi_robot_challenge_23',
        #     executable='marker_detection_node',
        #     namespace='tb3_6',
        #     name='marker_detection_node'),
        # launch_ros.actions.Node(
        #     package='multi_robot_challenge_23',
        #     executable='marker_detection_node',
        #     namespace='tb3_7',
        #     name='marker_detection_node'),
        # launch_ros.actions.Node(
        #     package='multi_robot_challenge_23',
        #     executable='marker_detection_node',
        #     namespace='tb3_8',
        #     name='marker_detection_node'),
        launch_ros.actions.Node(
            package='scoring',
            executable='scoring',
            name='scoring'),
        launch_ros.actions.Node(
            package='multi_robot_challenge_23',
            executable='coordination',
            name='coordination'),
    ])
