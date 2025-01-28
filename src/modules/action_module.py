import sys
import qi
from time import sleep, time
import re
import numpy as np
import random


# During development I won't set up the QI_URL as an environment variable - will be added later
# QI_URL = os.environ["QI_URL"] 
#QI_URL = "127.0.0.1:9559"
QI_URL = "192.168.50.231:9559"

COORD_PATTERN = r'(?<=\[)[-?\d\s,]+(?=\])'

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

class ActionGeneration():
    """
        The action generation module for the NAO
    """

    def __init__(self):

        #QI_URL = os.environ["QI_URL"] 
        #QI_URL = "192.168.50.231:9559"
        #QI_URL = "127.0.0.1:45113"

        # This will be used later as a regex function to filter out coordinates from the llm response with the format [x, y, z]

        self.qiapp = qi.Application(url=QI_URL)
        self.session = self.qiapp.session

        self.qiapp.start()

        self.maxSpeed = 0.2

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        self.tts = self.session.service("ALTextToSpeech")
        self.tts.setParameter("speed", 80)

        self.tracker = self.session.service("ALTracker")
        self.bm = self.session.service("ALBehaviorManager")
        self.memory = self.session.service("ALMemory")

        self.posture = self.session.service("ALRobotPosture")
        self.posture.goToPosture("Sit", 1.0)

        self.motion = self.session.service("ALMotion")

        self.state_indicator = self.session.service("ALLeds")

        

    def talker(self, message: str):
        """Says the message given to the NAO"""
        self.tts.say(message)
        sleep(1)


    def updateCoordinates(self,x,y,z):
        """Function that simply updates Parameters"""
        self.x = x
        self.y = y
        self.z = z
    

    def onCallLook(self, *_args):
        """Lets NAO look to a certain position"""

        self.updateCoordinates(*_args)

        self.useWholeBody = False
        self.frame = 0 #0 - TORSO, 1 - World, 2- Robot

        self.tracker.lookAt([self.x, self.y, self.z], self.frame, self.maxSpeed, self.useWholeBody)

    def shaking_head(self, *args):
        self.motion.setStiffnesses("Head", 1.0)
            
        # Define the angles for shaking
        left_angle = -0.7
        right_angle = 0.7

        # Calculate the end time
        end_time = time() + 2
        speed = 0.6
        
        while time() < end_time:
            # Move the head to the left
            self.motion.setAngles("HeadYaw", left_angle, speed)
            sleep(0.4)
            
            # Move the head to the right
            self.motion.setAngles("HeadYaw", right_angle, speed)
            sleep(0.4)

            
        # Return the head to the neutral position
        self.motion.setAngles("HeadYaw", 0.0, speed)
        sleep(0.5)
        
        # Relax the head
        self.motion.setStiffnesses("Head", 0.0)


    def nodding(self, *args):
        self.motion.setStiffnesses("Head", 1.0)
            
        # Define the angles for shaking
        up_angle = -0.4
        down_angle = 0.4

        # Calculate the end time
        end_time = time() + 2
        speed = 0.5
        
        while time() < end_time:
            # Move the head to the left
            self.motion.setAngles("HeadPitch", up_angle, speed)
            sleep(0.35)
            
            # Move the head to the right
            self.motion.setAngles("HeadPitch", down_angle, speed)
            sleep(0.35)

            
        # Return the head to the neutral position
        self.motion.setAngles("HeadPitch", 0.0, speed)
        sleep(0.5)
        
        # Relax the head
        self.motion.setStiffnesses("Head", 0.0)


    def onCallPoint(self, *_args):
        """Lets NAO point to a certain position"""
        self.updateCoordinates(*_args)

        self.effector = "RArm"
        self.frame = 0 #0 - TORSO, 1 - World, 2- Robot
        self.tracker.pointAt(self.effector, [self.x, self.y, self.z], self.frame, self.maxSpeed)
        sleep(1)

    def think_step_by_step(self, *args):
        """This is a dummy function for the LLM to have a chain of thought implemented without putting it to the output"""
        pass

    def thinking(self):
        self.state_indicator.fadeRGB("FaceLeds", "yellow", 0.3)
        self.state_indicator.fadeRGB("EarLeds", "yellow", 0.1)
    
    def stop_thinking(self):
        self.state_indicator.fadeRGB("FaceLeds", "green", 0.3)

    def listening(self):
        self.state_indicator.fadeRGB("FaceLeds", "white", 0.3)
        self.state_indicator.fadeRGB("EarLeds", "white", 0.1)

    def error(self):
        self.state_indicator.fadeRGB("FaceLeds", "red", 0.3)




    def executor(self, coords):
        """Execution of the point and stare task"""
        try:
            if not isinstance(coords[0], int):
               new_coords = [float(coord) for coord in coords[0]] # sometimes, GPT-4 gives the coordinates in the format [[x, y, z]] as a nested list. If so, only take the first member. 
            else:
                new_coords = [float(coord) for coord in coords]
            self.onCallLook(*new_coords)
            self.onCallPoint(*new_coords)
        except (TypeError, NameError, IndexError) as e:
            self.error()
            print(e)
            self.talker(random.choice(funny_excuses))
            self.listening()


    def stop(self):
        """Stop of the Application"""
        self.qiapp.stop()


    def shutdown(self):
        """Shutdown of the brokertester"""
        self.stop()
        sys.exit(0)

def main():
    action_generation = ActionGeneration()
    #action_generation.talker("Ich möchte Löwenbändiger werden")

    #point = [1.380, 0.124, -0.450]
    #point = [826.0, -480.1807372175981, -216.3622175980976]
    #action_generation.executor(point)    
    #action_generation.nodding()
    action_generation.posture.goToPosture("Sit", 1.0)
    #action_generation.nodding()
    #action_generation.shaking_head()
    pass

if __name__ == "__main__":
    main()