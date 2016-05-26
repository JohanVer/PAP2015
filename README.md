#PAP2015


![Image of PAP2015](https://github.com/JohanVer/PAP2015/blob/master/pap2015.jpg)
#TODO

Johan:
* Dispenser-Planner: Modify dispenser height
* CHECKED Background image fix 
* CHECKED Calibration button should not make a difference by pressing it twice
* CHECKED All in one window calibration
* Implement Dot planner and add parameters to gui
* Saving all parameters and calibration value in a xml file
* Documentation: PCB_CV (Algorithms, Stitcher...)
* Documentation: MOTOR_CONTROLLER (Low Level, Interface)
* Documentation: PAD_FINDER_LEARNER
* Documentation: ARDUINO

Nikolas:
* OBSOLETE: Placer-Planner: Wait until all motors have reached the position before starting the next task
* Calibration: Implement Dispenser Needle calibration
* CHECKED: Placer-Planner: Implement the second nozzle
* Placer-Planner: Implement complete (all parts with one click magic) assembling of smd parts
* CHECKED: Placer-Planner: Test the part rotation procedure (does the system rotate the part in the right direction?)
* CHECKED: Placer-Planner: Placing with both nozzles. Checking if actual nozzles are compatible to parts which should be assembled
* Documentation: PAP_PLACER
* Documentation: PAP_GUI
* Documentation: ROBOT_SIMULATION

Both:

* Slides


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

INSTALL ZBAR:
sudo apt-get install libzbar-dev libdmtx-dev

For using the image_simulator you should add something like this to your bashrc:
export PAPRESOURCES=/home/johan/Dokumente/catkin_ws/src/PAP2015/PAP/
