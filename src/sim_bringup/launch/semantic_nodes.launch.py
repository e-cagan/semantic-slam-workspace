"""
Launch file that runs the implemented nodes
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Function that generates launch decription.
    """

    return LaunchDescription([

        # Perception node
        Node(
            package='perception',
            executable='perception_node',
            name='perception_node',
            parameters=[{
                'yolo_model_path': 'yolov8n.pt',
                'conf_threshold': 0.05,                 # Low because YOLO can't detect otherwise in gazebo sim
                'image_size': 640,
            }]
        ),

        # Semantic map node
        Node(
            package='semantic_map',
            executable='semantic_map_node',
            name='semantic_map_node',
            parameters=[{
                'distance_threshold': 0.5,
            }]
        ),

        # Semantic visualizations node
        Node(
            package='semantic_viz',
            executable='semantic_viz_node',
            name='semantic_viz_node',
        ),

    ])