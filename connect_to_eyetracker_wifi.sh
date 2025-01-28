# Disconnect from potential wifi
sudo nmcli device disconnect wlx2887ba780f2d

# Connect to Eyetracker-Wifi
sudo nmcli device wifi connect "TG03B-080201136311" password "TobiiGlasses" ifname wlx2887ba780f2d
