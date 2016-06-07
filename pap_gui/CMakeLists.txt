##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 3.1)
project(pap_gui)

include (/usr/dlib-18.18/dlib/cmake)

set(CMAKE_BUILD_TYPE Debug)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

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
  pcb_cv
  pap_placer
  tf
  motor_controller
)

find_package( OpenCV 3.1 REQUIRED )

# Find the QtWidgets library
set(CMAKE_PREFIX_PATH $ENV{HOME}/Qt/5.6/gcc_64)
set(CMAKE_INCLUDE_PATH $ENV{HOME}/Qt/5.6/gcc_64)

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)

find_package(Qt5Widgets)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pap_gui
  CATKIN_DEPENDS roscpp rospy std_msgs image_transport cv_bridge sensor_msgs pap_common pcb_cv pap_placer tf motor_controller
  DEPENDS system_lib
)

QT5_WRAP_UI(QT_FORMS_HPP ${QT_FORMS} ui/main_window.ui ui/slotselectordialog.ui ui/versionSelectorDialog.ui ui/packageDialog.ui ui/tapecalibration.ui)

add_library(pap_gui
   src/versionSelectorDialog.cpp
   src/packageDialog.cpp
   src/slotselectordialog.cpp
   src/main_window.cpp
   src/ClickGraphicsView.cpp
   src/qnode.cpp
   src/DatabaseClass.cpp
   src/SlotGraphicsView.cpp
   src/MyContextMenuTable.cpp
   src/DispenserPlanner.cpp
   src/PlacementPlanner.cpp
   src/GerberPadParser.cpp
   src/PadView.cpp
   src/stitchWaypointMaker.cpp
   src/tapecalibration.cpp
   ${QT_FORMS_HPP}
   include/pap_gui/ClickGraphicsView.hpp
   include/pap_gui/main_window.hpp
   include/pap_gui/SlotGraphicsView.hpp
   include/pap_gui/MyContextMenuTable.hpp
   include/pap_gui/slotselectordialog.h
   include/pap_gui/DatabaseClass.hpp
   include/pap_gui/packageDialog.hpp
   include/pap_gui/versionSelectorDialog.h
   include/pap_gui/DispenserPlanner.hpp
   include/pap_gui/PlacementPlanner.hpp
   include/pap_gui/PadView.hpp
   include/pap_gui/GerberPadParser.hpp
   include/pap_gui/qnode.hpp
   include/pap_gui/stitchWaypointMaker.h
   include/pap_gui/tapecalibration.h
 )

target_link_libraries(pap_gui Qt5::Widgets dlib ${OpenCV_LIBS} ${catkin_LIBRARIES})
include_directories(${catkin_INCLUDE_DIRS} include ${OpenCV_INCLUDE_DIRS})

##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)
#file(GLOB_RECURSE INCLUDE_GLOBAL_HEADERS ${PROJECT_SOURCE_DIR} *.h *.hpp)
##############################################################################
# Binaries
##############################################################################

add_executable(pap_gui_node src/main.cpp )
target_link_libraries(pap_gui_node pap_gui Qt5::Widgets ${catkin_LIBRARIES})

