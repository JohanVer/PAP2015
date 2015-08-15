# PAP2015

DEPENDENCIES:
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
