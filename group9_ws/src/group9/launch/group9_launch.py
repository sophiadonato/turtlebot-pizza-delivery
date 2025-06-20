import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='group9',
            executable='aruco_subscriber',
            name='aruco_subscriber'),
        launch_ros.actions.Node(
            package='group9',
            executable='robot_pose',
            name='robot_pose'),
        launch_ros.actions.Node(
            package='group9',
            executable='delivery',
            name='delivery'),
        launch_ros.actions.Node(
            package='group9',
            executable='raycaster',
            name='raycaster'),
        launch_ros.actions.Node(
            package='group9',
            executable='map_explorer',
            name='map_explorer'),
  ])