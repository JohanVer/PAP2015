TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp

INCLUDEPATH += /usr/opencv3/include/opencv
INCLUDEPATH += /usr/opencv3/include/opencv2

QMAKE_CXXFLAGS = -L/usr/opencv3/lib \
       -I/usr/opencv3/include

LIBS += -L/usr/opencv3/lib
LIBS += -lopencv_core
LIBS += -lopencv_imgproc
LIBS += -lopencv_highgui
LIBS += -lopencv_ml
LIBS += -lopencv_video
LIBS += -lopencv_features2d
LIBS += -lopencv_calib3d
LIBS += -lopencv_objdetect
LIBS += -lopencv_contrib
LIBS += -lopencv_legacy
LIBS += -lopencv_flann
LIBS += -lopencv_nonfree
LIBS += -lopencv_imgcodecs
LIBS += -lopencv_stitching
LIBS += -lopencv_reg
