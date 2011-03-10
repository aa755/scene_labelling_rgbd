cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Release)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()
#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
##############################################################################
# Required 3rd party
##############################################################################
# http://qtnode.net/wiki/Qt4_with_cmake

find_package(Qt4 REQUIRED)
SET(QT_USE_QTXML TRUE)
#SET(QT_USE_QTOPENGL TRUE)
include(${QT_USE_FILE})
# This is necessary as all ui files etc will get dumped in the bottom of the
# binary directory.
include_directories(${CMAKE_CURRENT_BINARY_DIR}) 
##############################################################################
# Sections
##############################################################################

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/local/*.hpp src/qtros.h src/openni_listener.h src/qtcv.h src/graph_manager.h)

QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/gicp-fallback.cpp src/main.cpp src/qtros.cpp  src/openni_listener.cpp src/qtcv.cpp src/flow.cpp src/node.cpp src/graph_manager.cpp)

##############################################################################
# Binaries
##############################################################################
rosbuild_add_executable(rgbdslam ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(rgbdslam ${QT_LIBRARIES})
