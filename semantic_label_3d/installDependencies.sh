echo "installing some dependencies .... if all steps succed, you will see Success!!  else see comments in this script for possible fixes"
set -e
#cd boost-numeric-bindings/
#echo "installing boost numeric bindings for gels least squares fitting and others"
#./configure
#make
#sudo make install
cd ..
sudo apt-get install subversion ros-electric-octomap-mapping ros-electric-perception-pcl ros-electric-perception-pcl-addons
#cd ..
#svn co https://code.ros.org/svn/wg-ros-pkg/branches/trunk_boxturtle/stacks/semantic_mapping
#rosstack find semantic_mapping # scene_labelling must be in ROS_PACKAGE_PATH
#cd semantic_mapping
#cd ANN
#make
#cd ../cminpack
#make

#cd ../FLANN
#make

#cd ../point_cloud_mapping
#make

#svn co http://svn.pointclouds.org/ros/trunk/perception_pcl_unstable/pcl pcl
#cd pcl
#make

echo "Success!!"

