Steps for Configuring Your GPS (for donkeycar or ROS2 coordinate publishing)
Huge thanks to Sid for helping me debug this.
Note that the GPS can only do coordinates in the donkeycar preferred format or ROS2 publishing format at any given time. However it is pretty quick to switch modes in the 
configuration tool.
Also note that there is a app made by PointOne Nav to do this configuration. However, this guide uses the config.py program that is shipped by the company.

Configuring the GPS for donkeycar:
1. cd projects
2. git clone https://github.com/PointOneNav/p1-host-tools.git
3. cd p1-host-tools/
4. python3 -m venv venv
	This is creating a virtual environment to avoid dependency conflicts with other programs.
5. source venv/bin/activate
	You should see (venv) to the left of your text after running this command, that means that you're in the virtual environment
6. pip3 install -e.
7. python3 bin/config_tool.py apply uart2_message_rate nmea gga on
	You may get errors saying ___ not found, if so do pip3 install ___ and rerun the program
8. python3 bin/config_tool.py save (after successfully running config_tool.py)

Then, you're done!
Ask the professor/TA for a polaris username and password for your group.
You can run python3 bin/runner.py --device-id <polaris_username> --polaris <polaris_password> --device-port /dev/ttyUSB1   
(if not getting any data including Nans try USB0)

Make sure to be outside while doing this, the gps will just output nan's while indoors.

This should display atmospheric distortion corrected gps coordinates.


Setting up ROS2 gps publishing to /fix and /gps_fix:

1. Start a docker container with the djnighti/ucsd_robocar: devel  image (you need the newer ubuntu version in the docker for later steps to work)
	Document 100 pg 17 has instructions on how to do this if you are confused
	“docker pull djnighti/ucsd_robocar:devel” is the command to download this image
2. Follow the instructions in https://github.com/PointOneNav/ros2-fusion-engine-driver/blob/main/README.md 
I will write them out here with a couple bug corrections for your convenience. Note that we are running ros2 foxy, not humble

3. Install some dependencies
apt-get install ros-foxy-gps-msgs
apt install ros-foxy-nmea-msgs
apt install ros-foxy-mavros

4. Configuring your device:
Navigate to the p1-host-tools that you set up earlier
source venv/bin/activate
python3 bin/config_tool.py apply uart2_message_rate fe ROSPoseMessage 100ms
python3 bin/config_tool.py apply uart2_message_rate fe ROSGPSFixMessage 100ms
python3 bin/config_too.py save

The website instructions include a 3rd command which is setting up the imu. However our gps devices do not have the firmware loaded to support that, so that command will not 
work.

5. git clone https://github.com/PointOneNav/ros2-fusion-engine-driver.git

6. cd ros2-fusion-engine-driver

7. rosdep install -i --from-path ./ --rosdistro foxy -y

8. build_ros2
	This will take a long time, about three minutes. If you get an error saying various c++ commands are not found it is the iomanip loading error. Normally, the iomanip 
library is automatically loaded into the c++ compiler, but for some reason it is not being recognized on ours. 
	To fix, navigate to the file fusion-engine-driver/utils/conversion_utils.hpp and add #include <iomanip> to the top of the file

9. To run the command, do
ros2 run fusion-engine-driver fusion_engine_ros_driver --ros-args -p connection_type:=tty -p tty_port:=/dev/ttyUSB1
We are connecting through serial. 

10. To check that the gps output is being published, you can open a new terminal, enter the same container that the gps is running in, and do ros2 topic echo /fix and you 
should see lots of gps coordinates being published.


Troubleshooting:
If you have version conflicts with the config.py, (specifically catkin version errors for me), try setting up the p1_tools inside of a docker container. The docker containers 
run a newer version of ubuntu linux which can help fix these conflicts.

If you have significant configuration issues, running  
python3 bin/config_tool.py reset factory 
and then reconfiguring can be a good idea to fix them.

If you decide you don't need the fusion-engine-client and want fast build times, I recommend modifying the build_ros2 command in the ~./bashrc to exlude the 
fusion-engine-driver package for faster loading time, unless you really need it. Excluding the ntrip_client can save time too. 

Here is the changed build_ros2 command
function build_ros2() {
  cd /home/projects/ros2_ws 
  rm -rf build/ install/ log/ 
  colcon build --packages-ignore fusion-engine-driver ntrip_client
  source install/setup.bash
}
make sure to source ~/.bashrc after to have the changes take effect
