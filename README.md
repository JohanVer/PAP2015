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


BUILD ARDUINO-MSGS:

rosrun rosserial_arduino make_libraries.py


BUILD ECLIPSE FILES:

First :
 catkin_make --force-cmake -G"Eclipse CDT4 - Unix Makefiles"

Then:
 awk -f $(rospack find mk)/eclipse.awk build/.project > build/.project_with_env && mv build/.project_with_env build/.project
