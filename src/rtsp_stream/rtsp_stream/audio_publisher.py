"""This node is to publish the Audio data from the microphone input via the network with the topic "audio".
It also contains a subscription to the "prevent_loopback" topic that intents to prevent a loopback between NAO and the transcription module"""

import sys
import os
sys.path.append(os.path.abspath(os.getcwd()))

import logging
import traceback
from time import sleep
import pyaudio
import re
from rclpy.duration import Duration

from src.modules.action_module import ActionGeneration


import speech_recognition as sr
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import numpy as np
from queue import Queue
import whisper
from src.modules.stt_model import whisper_stt


def comparison(words1, words2):
    """A small function that compares the input of the prevent_loopback topic.
    The purpose is that the publisher doesn't publish transcriptions from the output of the NAO"""
    if words2 == "":
        return words1
    # Split sentences into words, including punctuation as separate entities
    words1 = re.findall(r'\b\w+\b|\S', words1)
    words2 = re.findall(r'\b\w+\b|\S', words2)
    
    # Remove specific punctuation marks from word lists
    words1 = [i.lower() for i in words1 if i not in [".", ",", ";", "-", "!", "?"]]
    words2 = [i.lower() for i in words2 if i not in [".", ",", ";", "-", "!", "?"]]

    word_list = [words1, words2]
    return word_list

class AudioPublisher(Node):
    """
    A ROS 2 node that publishes audio transcriptions as messages.

    This class defines a ROS 2 node that publishes text messages representing audio transcriptions. It initializes
    the ROS node, sets up a publisher, and includes methods to publish messages and log information.

    Methods:
        __init__(): Initializes the AudioPublisher node, sets up the publisher, and logs the start message.
        publish(text): Publishes a text message to the 'audio' topic.
        add_msg_to_info_logger(msg): Logs an informational message using the node's logger.
    """
    def __init__(self):
        """
        Initializes the AudioPublisher node.

        Sets up the ROS 2 node with the name 'AudioPublisher', creates a publisher for the 'audio' topic, and logs
        an informational start message.
        """
        super().__init__("AudioPublisher")
        self.add_msg_to_info_logger("Starting audio publisher")
        self.publisher = self.create_publisher(String, 'audio', 10)
        timer_period = 0.05
        self.create_timer(timer_period, self.publish_transcription)

        self.loopback_subscription = self.create_subscription(Bool, 'prevent_loopback', self.prevent_loopback, 10)
        self.language_subscription = self.create_subscription(String, 'key_input', self.set_language, 10)

        self.language = "e"

        self.transcribe = False

        self.loopback_subscription  # prevent unused variable warning
        #self.language_subscription

        self.old_transcription = ''
        self.current_time = self.get_clock().now()

        self.allow_transcription = False  # Initially allow transcription
        self.transcription_block_until = self.get_clock().now()  # Time until transcription is blocked
        self.min_publish_interval = 8.0  # Minimum 5 seconds between transcriptions


        global activity
        activity = ActionGeneration()

        
    def block_transcription(self, duration):
        """Block transcription for a specified duration in seconds."""
        block_duration = Duration(seconds=duration)
        self.transcription_block_until = self.get_clock().now() + block_duration
        self.allow_transcription = False

    def check_transcription_status(self):
        """Update transcription status based on current time."""
        if self.get_clock().now() >= self.transcription_block_until and self.transcribe:
            self.allow_transcription = True


    def publish_transcription(self):
        """
        Publishes a text message to the 'audio' topic if transcription is allowed.

        Checks the transcribe string and publishes the message if allowed.
        """

        if self.transcribe:
            if not data_queue.empty() and self.transcribe:
                message = whisper_stt(audio_model, data_queue, self.transcribe, self.language)
                if message is not None:
                    word_list = comparison(message, self.old_transcription)
                    activity.thinking()

                    if len(word_list) > 0:
                        msg = String()

                        if word_list[0] == word_list[1] or (word_list[0] == ""):
                            msg.data = ""
                            self.publisher.publish(msg)
                            self.add_msg_to_info_logger(f"Sent: \"{message}\"")
                        elif word_list[1] == "":
                            msg.data = word_list[0].lower()
                            self.publisher.publish(msg)
                            self.add_msg_to_info_logger(f"Sent: \"{message}\"")
                        else:
                            msg.data = message.lower()
                            self.publisher.publish(msg)
                            self.add_msg_to_info_logger(f"Sent: \"{message}\"")

                        self.block_transcription(self.min_publish_interval)
            else:
                sleep(0.05)
        else:
            sleep(0.05)


    def prevent_loopback(self, msg):
        """
        Updates the transcribe string based on the incoming message.

        Args:
            msg (String): The incoming string message that contains what the robot said in order to prevent loopback.
        """

        self.transcribe = msg.data
    
    def set_language(self, msg):
        if msg.data == "e" or msg.data == "s" or msg.data == "b":
            self.language = msg.data

    def add_msg_to_info_logger(self, msg):
        """
        Logs an informational message using the node's logger.

        Args:
            msg (str): The message to be logged.
        """
        self.get_logger().info(msg)        


def main(args=None):
    rclpy.init(args=args)

    audio_publisher = AudioPublisher()

    # The data queue have to be global since it is also used in the audio_publisher node
    global data_queue

    data_queue = Queue()

    recorder = sr.Recognizer()
    recorder.energy_threshold = 5000
    recorder.dynamic_energy_threshold = False

    def play_audio(audio_data):
        """Function to monitor microphone input while transcribing - will be implemented in the final Framework"""
        # Initialize PyAudio
        p = pyaudio.PyAudio()
        # Open a stream for playback
        stream = p.open(format=p.get_format_from_width(width=2),
                        channels=1,
                        rate=16000,
                        output=True,
                        input_device_index=31)
        # Play the audio
        stream.write(audio_data)
        # Close the stream
        stream.stop_stream()
        stream.close()
        p.terminate()

    source = None

    # This loop takes the default Microphone on the pulseaudio server running on the system
    for index, name in enumerate(sr.Microphone.list_microphone_names()):

        # if "TobiiGlasses_Microphone" in name:
        #     try:
        #         source = sr.Microphone(sample_rate=16000, device_index=index)
        #         break
        #     except Exception as e:
        #         print(f"Could not initialize microphone '{name}': {e}")
        #         traceback.print_exc()
        #         exit(1)
        if "pipewire" in name:
            try:
                source = sr.Microphone(sample_rate=16000, device_index=index)
                break
            except Exception as e:
                print(f"Could not initialize microphone '{name}': {e}")
                traceback.print_exc()
                exit(1)

    # If there is no pulseaudio running on the system, take the default microphone from the default audio server
    if source is None:
        try:
            source = sr.Microphone()
        except Exception as e:
            logging.error(f"Failed to initialize default microphone: {e}")
            exit(1)

            
    # This has been used for the docker - will be available again in the final Framework
    # if os.environ["MICROPHONE_INDEX"] != "":
    #     source = sr.Microphone(sample_rate=16000, device_index=os.environ["MICROPHONE_INDEX"])

    model = "base"
    global audio_model
    audio_model = whisper.load_model(model)

    def record_callback(_, audio: sr.AudioData) -> None:
        """
        Processes audio data and places it into a queue.

        Clears the queue if the last publish was less than 5 seconds ago.
        """
        audio_publisher.check_transcription_status()

        if audio_publisher.allow_transcription:
            # Add new audio data to the queue
            data = audio.get_raw_data()
            data_queue.put(data)
        else:
            data = audio.get_raw_data()
            #audio_publisher.add_msg_to_info_logger("Transcription blocked.")
            # Uncomment the following if playback is needed

            #play_audio(data)

    # Calibrating the microphone to adjust for ambient noise
    with source:
        print("Calibrating microphone, this takes some seconds...")
        recorder.adjust_for_ambient_noise(source, duration=3)
    
    activity.listening()

    recorder.listen_in_background(source, record_callback, phrase_time_limit=3)

    audio_publisher.add_msg_to_info_logger("Now listening...")

    rclpy.spin(audio_publisher)

    audio_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
