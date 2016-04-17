##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.11)
project(pap_gui)

##############################################################################
# Catkin
##############################################################################

# qt_build provides the qt cmake glue, roscpp the comms for a default talker
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  image_transport
  cv_bridge
  sensor_msgs
  pap_common
  pap_placer
  tf
)

# Find the QtWidgets library
set(CMAKE_PREFIX_PATH $ENV{HOME}/Qt/5.6/gcc_64)
set(CMAKE_INCLUDE_PATH $ENV{HOME}/Qt/5.6/gcc_64)

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)

find_package(Qt5Widgets)
find_package(Qt4 COMPONENTS QtCore QtGui QtOpenGL REQUIRED)

include_directories(${catkin_INCLUDE_DIRS} include)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs image_transport cv_bridge sensor_msgs pap_common pap_placer tf
  DEPENDS system_lib
)

file(GLOB_RECURSE INCLUDE_GLOBAL_HEADERS ${PROJECT_SOURCE_DIR} *.h *.hpp)


set(qt_srcs
      src/versionSelectorDialog.cpp src/packageDialog.cpp src/slotselectordialog.cpp src/main_window.cpp)
set(qt_hdrs
      include/pap_gui/slotselectordialog.h include/pap_gui/versionSelectorDialog.h include/pap_gui/packageDialog.hpp include/pap_gui/main_window.hpp)

QT5_WRAP_UI(QT_FORMS_HPP ${QT_FORMS} ui/main_window.ui ui/slotselectordialog.ui ui/versionSelectorDialog.ui ui/packageDialog.ui)


##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

##############################################################################
# Binaries
##############################################################################

add_executable(pap_gui ${QT_SOURCES} ${uis_h} ${qt_srcs} ${QT_FORMS_HPP} ${INCLUDE_GLOBAL_HEADERS} src/MyContextMenuTable.cpp src/GerberPadParser.cpp src/SlotGraphicsView.cpp src/PadView.cpp src/slotselectordialog.cpp src/DatabaseClass.cpp src/versionSelectorDialog.cpp src/packageDialog.cpp)
target_link_libraries(pap_gui Qt5::Widgets ${catkin_LIBRARIES})

