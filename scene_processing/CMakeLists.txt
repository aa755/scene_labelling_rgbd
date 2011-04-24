cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()


#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()
#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_executable (scene_segmentation src/segmentation.cpp)
rosbuild_add_executable (scene_labler src/labeler.cpp)
#rosbuild_add_executable (label_visualizer src/labelVisualizer.cpp)
rosbuild_add_executable (merge_frames src/merge_frames.cpp)
#rosbuild_add_executable (shape_features src/shape_features.cpp)
rosbuild_add_executable (apply_labels src/applyLabels.cpp)
#rosbuild_add_executable (shape_features src/shape_features.cpp)
rosbuild_add_executable (compute_features src/compute_features.cpp)
rosbuild_add_executable (compute_all_features src/compute_all_features.cpp)
rosbuild_add_executable (transform src/transform.cpp)
rosbuild_add_executable (transformInteractive src/transformInteractive.cpp)
#rosbuild_add_executable (compute_neighbors src/get_neighbors.cpp)
#rosbuild_add_executable (extract_frames src/extract_frames.cpp)
#rosbuild_add_executable (mls src/mls.cpp)
#target_link_libraries ( pcl_kdtree pcl_surface)

rosbuild_add_executable (combineVisibility src/VisibilityMerge.cpp)
#rosbuild_add_executable (combineNearestCam src/NearestCamMerge.cpp)
