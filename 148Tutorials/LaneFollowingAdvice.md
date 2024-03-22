General pieces of advice for getting line and lane following working, from some late-night testing that Alexander and Winston did.

First thing, after you run the camera calibration program, make sure to run build_ros2 afterward to ensure your configuration is saved. The way the calibration program works 
is it autosaves whatever setup it has to a configuration file when you quit it. If you accidentally run it a second time without running build_ros2, it will just load the 
default settings, and then overwrite your hard-earned configuration when you quit it.

Lighting is really really really important for having line/lane following working well. You want even lighting over the track. Noon and night are both bad, 
morning/afternoons where the track is uniformy shaded are good.

The calibration program is pretty laggy just because it has to send so much data to your computer, and does not work reliably for configuring 
the vesc/pid settings. Instead, I recommend configuring those directly in the file, which is located in 
/home/projects/ros2_ws/src/ucsd_robocar_hub2/ucsd_robocar_lane_detection2_pkg/config

The just run the camera_nav node with the robot on a stand to test your settings.

If you have wifi issues, connecting to your jetson via usb can be a good way to get around them. I did my successful lane/line following attempts with my computer tethered to 
the robocar with a microusb cable.

Note that for some reason the vesc output values below ~900 rpm don’t seem to actually move the robot, and values above 900 move the robot much slower than you’d expect. This 
is an issue I don’t understand perfectly but I figured I’d mention. 


