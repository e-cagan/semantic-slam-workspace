"""
Launch file for implementing SLAM to localize and create a map afterwards.
"""

import os

# Set tb3 model
os.environ['TURTLEBOT3_MODEL'] = 'waffle'

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Function to generate launch description.
    """

    # World path
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    world = os.path.join(
        get_package_share_directory('sim_bringup'),
        'worlds',
        'semantic_slam.world'
    )
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='-2.0')
    y_pose = LaunchConfiguration('y_pose', default='-0.5')


    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )

    return LaunchDescription([
        # Simulator commands
        gzserver_cmd,
        gzclient_cmd,
        robot_state_publisher_cmd,
        spawn_turtlebot_cmd,

        # Open slam toolbox launch file (use simulation time)
        # IncludeLaunchDescription(
        #    PythonLaunchDescriptionSource(
        #        os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        #    ),
        #    launch_arguments={'use_sim_time': 'true'}.items()
        # ),

        # Slam and nav2 can't work simultaneously together

        # Open rviz node to visualize
        # Node(
        #    name='rviz_node',
        #    package='rviz2',
        #    executable='rviz2',
        #    parameters=[{'use_sim_time': True}]
        # ),

        # Also, rvizes are conflicting in here and in nav2 launch file

        # Open teleop node to move robot
        Node(
            name='teleop_twist_keyboard',
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            output='screen',
            prefix='xterm -e',
        ),
    ])