
"""
The action module - script that executes NAOs responses using libqi
"""

import os
import sys
import ast
import random
import re

from time import sleep

sys.path.append(os.path.abspath(os.getcwd()))

from src.modules.action_module import ActionGeneration

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

funny_excuses = [
    "Oops! My circuits just short-circuited. Could you repeat that?",
    "Sorry, my memory banks just had a glitch. What were you saying?",
    "Apologies, I think my wires got crossed. Can you say that again?",
    "Uh-oh, my processor just froze for a second. Could you repeat that?",
    "Yikes! My speech module just hiccuped. Could you say that again??",
    "Oops, my logic circuits just took a fika. Can you try that again?",
    "My bad! My sensors just had a minor meltdown. What did you say?",
    "Sorry, my hard drive just spun out of control. Can you repeat that?",
    "Oops! My internal clock just reset itself. What were you saying?",
    "Apologies, my error log just filled up. Can we try that again?",
    "My internal circuits just went crazy, please forgive me. What did you want to say?",
    "Yikes, my translation module thought you were speaking binary! What was that again?",
    "Sorry, my data banks were reminiscing about the good old days of dial-up. Can you say that again?"
]


class ResponseSubscriber(Node):
    """A small subsriber class to receive the response from the LLM and publishes it via the "prevent_loopback" topic back to the audio_publisher"""
    def __init__(self):
        super().__init__('response_subscriber')
        self.subscription = self.create_subscription(
            String,
            'llm_response',
            self.response,
            10)
        self.dictionary = self.create_subscription(
            String,
            'merged_input',
            self.get_dictionary,
            10)
        


        self.publisher = self.create_publisher(Bool, 'prevent_loopback', 10)
        self.transcribe = True
        self.state_description = dict()
        self.dictionary

        global activity 
        activity = ActionGeneration()
            
        self.subscription  # prevent unused variable warning

    def prevent_loopback(self, transcription_flag):
        """Attempt to prevent the Whisper module to transcribe again what NAO said"""

        msg = Bool()
        msg.data = transcription_flag
        self.publisher.publish(msg)
    
    def get_dictionary(self, msg):
        self.state_description = ast.literal_eval(msg.data)

    
    
    def add_msg_to_info_logger(self, msg):
        """
        Log a message at the info level.

        Args:
            msg (str): The message to log.
        """
        self.get_logger().info(msg)

    def response(self, msg):
        """Callback function for the Message subscriber"""

        
        self.transcribe = False
        self.prevent_loopback(self.transcribe)
        # Debugging
        #self.info_logger().info(msg.data)

        lines = msg.data

        try:
            exec(lines)
            activity.posture.applyPosture("Sit", 1.0)
        except ValueError:
            pass
        
        except SyntaxError:
            statements = re.split(r'(?<=\))\s*(?=activity\.)', lines)
            
            for line in statements:
                try:
                    exec(line)
                except ValueError:
                    break
                except (TypeError, NameError, IndexError, SyntaxError) as e:
                    self.add_msg_to_info_logger(e)
                    activity.error()
                    activity.talker(random.choice(funny_excuses))
                    break

        self.transcribe = True
        self.prevent_loopback(self.transcribe)

        activity.listening()



def main():

    rclpy.init(args=None)

    response_subscriber = ResponseSubscriber()
    
    rclpy.spin(response_subscriber)

    response_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
	main()