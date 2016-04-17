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

# Use this to define what the package will export (e.g. libs, headers).
# Since the default here is to produce only a binary, we don't worry about
# exporting anything. 
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs image_transport cv_bridge sensor_msgs pap_common pap_placer tf
  DEPENDS system_lib
)

##############################################################################
# Qt Environment
##############################################################################

# this comes from qt_build's qt-ros.cmake which is automatically 
# included via the dependency call in package.xml
#rosbuild_prepare_qt4(QtCore QtGui) # Add the appropriate components to the component list here

##############################################################################
# Sections
##############################################################################

file(GLOB_RECURSE INCLUDE_GLOBAL_HEADERS ${PROJECT_SOURCE_DIR} *.h *.hpp)
#file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
#file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
#file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/pap_gui/*.hpp)

set(qt_srcs
      src/versionSelectorDialog.cpp src/packageDialog.cpp src/slotselectordialog.cpp src/main_window.cpp)
set(qt_hdrs
      include/pap_gui/slotselectordialog.h include/pap_gui/versionSelectorDialog.h include/pap_gui/packageDialog.hpp include/pap_gui/main_window.hpp)
#qt5_automoc(${qt_srcs})
#QT5_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT5_WRAP_UI(QT_FORMS_HPP ${QT_FORMS} ui/main_window.ui ui/slotselectordialog.ui ui/versionSelectorDialog.ui ui/packageDialog.ui)
#QT5_WRAP_CPP(QT_MOC_HPP ${QT_MOC} ${qt_hdrs})

##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

##############################################################################
# Binaries
##############################################################################

#add_executable(pap_gui ${QT_SOURCES} ${QT_RESOURCES_CPP} ${uis_h} ${qt_srcs}  ${QT_FORMS_HPP} ${QT_MOC_HPP} ${INCLUDE_GLOBAL_HEADERS} src/MyContextMenuTable.cpp src/GerberPadParser.cpp src/SlotGraphicsView.cpp src/PadView.cpp src/slotselectordialog.cpp src/DatabaseClass.cpp src/versionSelectorDialog.cpp src/packageDialog.cpp)
add_executable(pap_gui ${QT_SOURCES} ${uis_h} ${qt_srcs} ${QT_FORMS_HPP} ${INCLUDE_GLOBAL_HEADERS} src/MyContextMenuTable.cpp src/GerberPadParser.cpp src/SlotGraphicsView.cpp src/PadView.cpp src/slotselectordialog.cpp src/DatabaseClass.cpp src/versionSelectorDialog.cpp src/packageDialog.cpp)
#target_link_libraries(pap_gui ${QT_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(pap_gui Qt5::Widgets ${catkin_LIBRARIES})
#install(TARGETS pap_gui RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

