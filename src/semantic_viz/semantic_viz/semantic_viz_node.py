"""
Module for semantic visualization node.
"""

import copy

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from visualization_msgs.msg import Marker, MarkerArray
from semantic_interfaces.msg import SemanticMap


class SemanticVizNode(Node):
    """
    A node that visualizes semantic observations on rviz.
    """

    def __init__(self):
        super().__init__('semantic_viz_node')

        # Qos
        self.qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        # Publishers
        self.marker_pub = self.create_publisher(MarkerArray, '/visualization_marker_array', 10)

        # Subscribers
        self.map_sub = self.create_subscription(SemanticMap, '/semantic/map_data', self.map_callback, self.qos)


    def map_callback(self, msg):
        """
        Callback function for map visualizations.
        """

        # Store index and create marker array to store marker informations
        idx = 0
        arr = MarkerArray()

        # Iterate trough all observations
        for obs in msg.observations:
            # Create marker msg and fill out it's fields
            marker = Marker()
            marker.type = Marker.SPHERE
            marker.pose = obs.pose.pose                                     # Pose, not PoseStamped
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.color.a = 1.0                                            # Don't forger the alpha value since color is RGBA in marker message
            marker.header.frame_id = 'map'
            marker.id = idx
            marker.ns = obs.label

            # Add text marker to display labels
            txt_marker = Marker()
            txt_marker.type = Marker.TEXT_VIEW_FACING
            txt_marker.text = obs.label
            txt_marker.pose = copy.deepcopy(obs.pose.pose)
            txt_marker.pose.position.z += 0.3                               # A bit up from marker pose
            txt_marker.scale.z = 0.2
            txt_marker.color.r = 0.0
            txt_marker.color.g = 0.0
            txt_marker.color.b = 1.0
            txt_marker.color.a = 1.0
            txt_marker.header.frame_id = 'map'
            txt_marker.id = idx + 1                                         # ID's can't be the same
            txt_marker.ns = obs.label

            # Add markers to marker array
            arr.markers.append(marker)
            arr.markers.append(txt_marker)

            # Increment the index
            idx += 2

        # Publish the marker array message
        self.marker_pub.publish(msg=arr)


# Main function to simulate node lifecycle
def main(args=None):
    """
    Main function that handles node lifecycle.
    """

    # Node lifecycle
    rclpy.init(args=args)
    node = SemanticVizNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


# Call the main function to run node
if __name__ == "__main__":
    main()