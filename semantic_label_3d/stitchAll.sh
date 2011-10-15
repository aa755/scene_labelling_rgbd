#!/bin/bash

for file in `dir -d *.bag` ; do
nohup roslaunch rgbdslam rgbdslam.launch &
rosbag play -r 0.1 $file
rosbag record -O finalStitching/$file.stitched.bag /rgbdslam/my_clouds /tf & 
sleep 360
killall -s SIGINT record
sleep 1
killall rgbdslam
sleep 1
done

