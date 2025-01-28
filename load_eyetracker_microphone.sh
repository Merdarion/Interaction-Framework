#!/bin/bash

# Create a virtual audio sink
if pactl list short sinks | grep -q "TobiiGlasses_Microphone"; then
    echo "'TobiiGlasses_Microphone' already exists."
else
    echo "Creating instance 'TobiiGlasses_Microphone'."
    pactl load-module module-null-sink sink_name=TobiiGlasses_Microphone sink_properties=device.description=TobiiGlasses_Microphone
    pactl load-module module-remap-source master="TobiiGlasses_Microphone.monitor" source_name="tobii_glasses_microphone" source_properties=device.description="tobii_glasses_microphone"
fi

# Set Tobii-Glasses as default source
pactl set-default-source tobii_glasses_microphone

# Capture audio from the RTSP stream
ffmpeg -rtsp_transport tcp -fflags nobuffer -acodec mp2 -buffer_size 100k -i rtsp://192.168.75.51:8554/live/all -preset ultrafast -map 0:1 -vn -ar 24000 -ac 1 -sample_fmt s16 -f pulse "TobiiGlasses_Microphone.monitor"