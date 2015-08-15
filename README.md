#PAP2015


![Image of PAP2015](https://github.com/JohanVer/PAP2015/blob/master/pap2015.jpg)
#TODO

Johan:
 
* Tape Calibration: Goto correct tape position (Lookup table)
* LED-Controller: Test new LED functions: Slider for dimming, blinking, color selection
* Dispenser-Planner: Sort dispenser tasks due to distances
* Dispenser-Planner: Check middle points of dispenser planning
* Dispenser-Planner: Modify dispenser height
* Dispenser-Planner: Pause-Button, Exclude-Function

Nikolas:

* Placer-Planner: Implement possibility to choose a tape
* Placer-Planner: Use the functions "calibrateTape" and "positionOfComponent"
* Placer-Planner: Check the parsed positions (after the point there are only zeros)
* Placer-Planner: Implement the second nozzle
* Placer-Planner: Implement complete (all parts with one click magic) assembling of smd parts
* Placer-Planner: Test the part rotation procedure (does the system rotate the part in the right direction?)

#DEPENDENCIES

sudo apt-get install ros-indigo-rosserial-arduino
sudo apt-get install ros-indigo-rosserial
sudo apt-get update && sudo apt-get install arduino arduino-core
sudo apt-get install ros-indigo-qt-create
sudo apt-get install ros-indigo-qt-build

INSTALL SERIAL PACKAGE:
(go into catkin_workspace/src)

git clone https://github.com/wjwwood/serial.git
make
make test
make install

INSTALL ZBAR PACKAGE:
(go into catkin_workspace/src)

git clone https://github.com/ZBar/ZBar.git
./configure (use automake to generate configure exe)
make
make check (optional)
make install

BUILD ARDUINO-MSGS:

rosrun rosserial_arduino make_libraries.py


BUILD ECLIPSE FILES:

First :
 catkin_make --force-cmake -G"Eclipse CDT4 - Unix Makefiles"

Then:
 awk -f $(rospack find mk)/eclipse.awk build/.project > build/.project_with_env && mv build/.project_with_env build/.project

START ARDUINO SERVER:
rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0

FastLED library Arduino:
Source: https://github.com/FastLED/FastLED/archive/3.0.3
Extract into sketchbook/libraries and give it a propper name like "FastLED"

For using the image_simulator you should add something like this to your bashrc:
export PAPIMAGES=/home/johan/Dokumente/catkin_ws/src/PAP2015/PAP/
