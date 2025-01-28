This repository consists of the NAO interaction Framework to interact with the NAO robot with auditive and visual cues.
It consists of six ROS-packages:

- rtsp_stream: publishes input information (scene camera and gaze overlay from the eye tracker and audio microphone input) via the network
- kinect_cameras: publishes rgb and depth image of the Azure Kinect cameras
- input_fusion: unifies all the input to publish it to the LLM
- llm_response: takes the input information, parses it into a GPT model and, then, publishes its response
- nao_action: gathers the llm response and generates actions

All functionality additional to just sending messages via the ROS network are outsourced into respective modules:
- action_module: stores the API to generate NAO actions, called in the actor node
- stt_model: Whisper STT model that is called in the audio_publisher node
- scene_description: an BLIP model for Image Captioning
- record_eyetracking_data: a script that records eye tracking data

And a pipeline that takes the scene_camera input and do some pre processing (called in the input_fusion node)
- object_detection_tracking: customized YOLO-World model with a ByteTrack algorithm
- object_segmentation: SAM2 model

These are in the subfolder "modules".

## Requirements: ##
- Python >= 3.10
- Ubuntu 22.04

**Installation of Azure Kinect Driver**
```
# Install Dependency first (libsoundio1) -> from an Ubuntu 20.04 repo
sudo apt-add-repository -y -n 'deb http://archive.ubuntu.com/ubuntu focal main'
sudo apt-add-repository -y 'deb http://archive.ubuntu.com/ubuntu focal universe'
sudo apt-get install -y libsoundio1

# Install Azure Kinect Driver
curl -sSL https://packages.microsoft.com/keys/microsoft.asc | sudo apt-key add -
sudo apt-add-repository https://packages.microsoft.com/ubuntu/18.04/prod
curl -sSL https://packages.microsoft.com/config/ubuntu/18.04/prod.list | sudo tee /etc/apt/sources.list.d/microsoft-prod.list
curl -sSL https://packages.microsoft.com/keys/microsoft.asc | sudo apt-key add -
sudo apt-get update
sudo apt install libk4a1.4
sudo apt install libk4a1.4-dev
sudo apt install libk4abt1.0-dev
sudo apt install k4a-tools
```
Afterwards, copy the file 99.k4a.rules in /etc/udev/rules/rules.d/ (and detach and reattach the device)! Otherwise it won't get recognized!

**Prerequisities before running pip install -r requirements.txt**

```
sudo apt install tmux
sudo apt install portaudio19-dev
```

**Before first use:**
- Install the necessary Python requirements from the requirements.txt file

**Important!**
The Framework requires setuptools with the version 69.5.1 in order to successfully build the ROS2 Python packages, but Ubuntu 22.04 comes with a newer version. A downgrade is required.

This Framework was developed in a virtual environment called "bident_framework". Hence, all the ROS2 packages point to the python executable in the folder of the virtual environment. Configuring this to point to the standard installation, erase the following lines from every "setup.cfg" file:

```
[build_scripts]
executable = ~/dev_bident/bin python3
```

## To run the Framework: ##

1. Open a terminal and run "load_eyetracker_microphone.sh"
2. Open a second terminal and run "run_framework.sh"
3. Additionally: To record eye tracking data, run "record_eyetracking_data.sh"

To **stop** the Framework:
- Open another terminal and run "kill_framework.sh"

Additionally, to stop the eye tracking recording:
- Go into the terminal that runs "record_eyetracking_data.sh" and hit ctrl+c

To stop the microphone of the Tobii Glasses of being streamed:
- Open another terminal and run "unload_eyetracker_microphone.sh"


**Please note: The first time the audio_publisher is running, the open source whisper module will be downloaded and stored in the "models" subfolder**


## The distinct ROS2 executables are:

**rtsp_stream**
1. audio_publisher
2. video_gaze_publisher (publishes scene camera as well as gaze data)

**kinect_cameras**
- kinect_publisher (publishes the RGB camera image as well as depth data

**keyboard_input**
- Keyboard publisher

**input_fusion**
- input_fusion

**llm_response**
- talker

**nao_action**
- actor

The Framework is still under development. Further information will be made available with the release.
