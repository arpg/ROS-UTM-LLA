This is a ros package for converting latitude, longitude and altitude coordinate system to UTM coordinate system (X,Y,Z in meters).
# Requirement
GeographicLib (test on version 1.34)
https://geographiclib.sourceforge.io/

# Download and build
```
mkdir -p catkin_ws/src
cd catkin_ws/src 
git clone https://github.com/arpg/ROS-UTM-LLA.git
cd ..
catkin_make
```

# Use
open a terminal and run roslaunch
```
roslaunch utm_lla coordinate_convertion.launch
```
Specify your UTM zone in the above launch file.  
if you want to convert LLA to UTM, publish your message to topic `/gps/fix`, ros message type `sensor_msgs::NavSatFix`. If you want to convert UTM to LLA, publish your message to topic `/utm/pose`, ros message type `geometry_msgs::PoseStamped`