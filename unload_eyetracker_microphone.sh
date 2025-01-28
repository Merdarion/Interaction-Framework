# Discard eyetracker microphone sink in ALSA
pactl unload-module module-null-sink
pactl unload-module module-remap-source
