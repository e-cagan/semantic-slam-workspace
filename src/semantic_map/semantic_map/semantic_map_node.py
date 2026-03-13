"""
Module for semantic mapping node.
"""

import numpy as np
import math

import rclpy
from rclpy.node import Node

from semantic_interfaces.msg import SemanticObservation
from semantic_interfaces.srv import SemanticQuery


class SemanticMapNode(Node):
    """
    A node that maps objects semantically.
    """

    def __init__(self):
        super().__init__('semantic_map_node')
        
        # Parameters
        self.declare_parameter("distance_threshold", 0.5)
        
        # Subscribers
        self.sem_sub = self.create_subscription(SemanticObservation, '/semantic/observations', self.observation_callback, 10)
        
        # Services
        self.query_srv = self.create_service(SemanticQuery, '/semantic/query', self.query_callback)
        
        # Internal state
        self.distance_threshold = self.get_parameter("distance_threshold").value
        self.semantic_map = dict()
    

    def observation_callback(self, msg):
        """
        A callback that observes.
        """

        # Check if label in semantic map dict or not. If not, add it
        if msg.label in self.semantic_map.keys():
            print(f"{msg.label} already in the dict.")

            for pose in self.semantic_map[msg.label]:
                # Calculate the distance
                dx = pose.pose.pose.position.x - msg.pose.pose.position.x
                dy = pose.pose.pose.position.y - msg.pose.pose.position.y
                dz = pose.pose.pose.position.z - msg.pose.pose.position.z
                distance = math.sqrt(dx**2 + dy**2 + dz**2)

                # Check the distance threshold
                if distance <= self.distance_threshold:
                    print("Already known.")
                    break
            else:
                self.semantic_map[msg.label].append(msg)
                print(f"New {msg.label} added.")
        else:
            self.semantic_map[msg.label] = [msg]
            print(f"Label {msg.label} added to the dict.")

    
    def query_callback(self, request, response):
        """
        Query service callback function.
        """

        # Search label in dict and return observations
        if request.object in self.semantic_map.keys():
            response.observations = self.semantic_map[request.object]
            return response
        else:
            return response


# Main function to simulate node lifecycle
def main(args=None):
    """
    Main function that handles node lifecycle.
    """

    # Node lifecycle
    rclpy.init(args=args)
    node = SemanticMapNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


# Call the main function to run node
if __name__ == "__main__":
    main()