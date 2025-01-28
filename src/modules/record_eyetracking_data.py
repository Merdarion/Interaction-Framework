import requests
import sys
from time import sleep


def set_meta_data(g3_address="192.168.75.51"):

    # Turn off gaze overlay
    url = "http://" + g3_address + "/rest/settings.gaze-overlay"
    body = False

    response = requests.post(url, json = body)

    # Print response
    if response.status_code == 200:
        print("Response:", response.json())
    else:
        print(f"Request failed with status code {response.status_code}")



def start_recording(g3_address="192.168.75.51"):
    url = "http://" + g3_address + "/rest/recorder!start"
    body = []

    response = requests.post(url, json = body)

    # Print response
    if response.status_code == 200:
        print("Recording started:", response.json())
    else:
        print(f"Request failed with status code {response.status_code}")


def stop_recording(g3_address="192.168.75.51"):
    url = "http://" + g3_address + "/rest/recorder!stop"
    body = []

    response = requests.post(url, json = body)

    # Print response
    if response.status_code == 200:
        print("Recording stopped:", response.json())
    else:
        print(f"Request failed with status code {response.status_code}")


if __name__ == "__main__":
    set_meta_data()
    start_recording()

    while True:
        try:
            sleep(0.05)
        except KeyboardInterrupt:
            stop_recording()
            sys.exit(0)
