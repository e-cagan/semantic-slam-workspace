"""
Module for llm navigator node.
"""

import transformers

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy

from nav2_msgs.action import NavigateToPose
from semantic_interfaces.srv import SemanticQuery
from std_msgs.msg import String


class LLMNavNode(Node):
    """
    A node that takes a query, sends it to LLM extracts the label and goes nearest label with nav2.
    """

    def __init__(self):
        super().__init__('llm_navigator_node')

        # Subscribers
        self.command_sub = self.create_subscription(String, '/llm_navigator/command', self.command_callback, 10)

        # Services
        self.query_client = self.create_client(SemanticQuery, '/semantic/query')
        
        # Check if the service is available and store the request
        while not self.query_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("Service not available. Trying again.")
        self.req = SemanticQuery.Request()

        # Actions
        self.nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Model pipeline
        self.pipe = transformers.pipeline(
            "text-generation",
            model="Qwen/Qwen2.5-0.5B-Instruct",
            device=0    # GPU
        )

    
    def send_request(self, object):
        """
        A helper method to send request to server.
        """

        self.req.object = object
        return self.query_client.call_async(self.req)
    

    def parse_command(self, command):
        """
        A helper function which parses user's message to extract label.
        """

        # Define system prompt to enject towards model
        system_prompt = """You are a robot navigation assistant. 
                            Extract only the target object label from the user's navigation command.
                            Return only the object name as a single word, nothing else.
                            Examples:
                            "Go to the chair" → chair
                            "Navigate to the person near the wall" → person
                            "Find the bottle on the table" → bottle"""
        
        # Unify messages
        messages = [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": command}
        ]

        # Take the output
        output = self.pipe(text_inputs=messages, max_new_tokens=20, return_full_text=False)     # return_full_text = False to return only output not with input

        # Return only label
        return output[0]['generated_text'].strip().lower()
    

    def query_response_callback(self, future):
        """
        Callback to handle query response.
        """

        # Take response and observations
        response = future.result()
        observations = response.observations

        # Check the observations are empty
        if len(observations) == 0:
            pass
        else:
            # Take first observation pose
            pose = observations[0].pose
            print(pose)


    def command_callback(self, msg):
        """
        Callback for commands.
        """

        # Get label and log it
        label = self.parse_command(msg.data)
        self.get_logger().info(f"Parsed label: {label}")

        # Send the request
        future = self.send_request(label)
        future.add_done_callback(self.query_response_callback)


# Main function to simulate node lifecycle
def main(args=None):
    """
    Main function that handles node lifecycle.
    """

    # Node lifecycle
    rclpy.init(args=args)
    node = LLMNavNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


# Call the main function to run node
if __name__ == "__main__":
    main()