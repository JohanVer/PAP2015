project(pad_finder_learner)
cmake_minimum_required(VERSION 2.8)

find_package( OpenCV 3 REQUIRED )

include (/usr/dlib-18.18/dlib/cmake)
include_directories(${OpenCV_INCLUDE_DIRS})

aux_source_directory(. SRC_LIST)
add_executable(learner ${SRC_LIST})

target_link_libraries ( learner dlib ${OpenCV_LIBS} )
