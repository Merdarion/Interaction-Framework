import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import ast

import traceback

import sys
import os
sys.path.append(os.path.abspath(os.getcwd()))


path = os.getcwd()

from src.llm_response.llm_response.prompt import get_prompt

import openai
from queue import Queue


client = openai.OpenAI(
    api_key = os.environ["OPENAI_API_KEY"]
)
message_queue = Queue()

class ResponsePublisher(Node):
    """
    A ROS 2 node that subscribes to the 'audio' topic, processes incoming messages using a language model, 
    and publishes responses to the 'llm_response' topic.

    Methods:
        __init__(self): Initializes the MessagePublisher node, sets up subscriptions and publishers, and starts the timer.
        receive_message(self, msg): Receives messages from the 'audio' topic and adds them to the message queue.
        process_queue(self): Processes messages in the queue, generates responses using a language model, and publishes them.
        llm_message(self, message): Generates a response for the given message using a language model.
    """

    def __init__(self):
        """
        Initializes the MessagePublisher node, sets up subscriptions and publishers, and starts the timer.
        """
        super().__init__('llm_response')

        self.subscription = self.create_subscription(
            String,
            'merged_input',
            self.receive_message,
            10)
        self.subscription  # prevent unused variable warning
        self.message_queue = Queue()

        self.publisher_ = self.create_publisher(String, 'llm_response', 10)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.process_queue)

        self.messages = get_prompt()




    def receive_message(self, msg):
        """
        Receives messages from the 'audio' topic and adds them to the message queue.

        Args:
            msg (std_msgs.msg.String): The message received from the 'audio' topic.
        """
        if msg.data != "":
            self.message_queue.put(msg.data)
            #self.get_logger().info(f"Got data! Response is being generated...")
            #self.get_logger().info(msg.data)



    def process_queue(self):
        """
        Processes messages in the queue, generates responses using a language model, and publishes them.
        """
        if not self.message_queue.empty():
            message = self.message_queue.get()
            processed_msg = self.llm_message(message)
            self.publisher_.publish(processed_msg)




    def llm_message(self, string_input):
        """
        Generates a response for the given message using a language model.

        Args:
            message (str): The message to process.

        Returns:
            std_msgs.msg.String: The generated response.
        """
        llm_response = String()

        self.messages.append({"role": "user", "content": string_input})

        if self.messages:
            response = ""
            try:
                chat = client.chat.completions.create(
                    model="gpt-4o-mini", messages=self.messages,
                    seed=0,
                    stream=True,
                    temperature=0,
                    logit_bias={"1734": -100} # Filter out \n
                )

                for chunk in chat:
                    if chunk.choices[0].delta.content is not None:
                        response += chunk.choices[0].delta.content
                        llm_response.data = response.strip()
                        if llm_response.data.strip() != "":
                            self.messages.append({"role": "assistant", "content": chunk.choices[0].delta.content})
                        else:
                            # Re-initializing chat if GPT-4o get's stuck
                            self.messages = get_prompt()

                self.get_logger().info(f'LLM: "{llm_response.data}"')
            except Exception as e:
                self.get_logger().error(f"OpenAI API call failed: {e}")
                self.get_logger().error(traceback.format_exc())


            return llm_response

        

def main(args=None):
    """
    Initializes the ROS 2 Python client library, creates a ResponsePublisher node, logs a message, 
    and spins the node to keep it alive.

    Args:
        args (list, optional): Command line arguments. Defaults to None.
    """
    rclpy.init(args=args)

    response_publisher = ResponsePublisher()

    response_publisher.get_logger().info("Now I'm listening...")    

    rclpy.spin(response_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    response_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()