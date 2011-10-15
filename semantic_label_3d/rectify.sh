rosrun scene_processing scene_labler $1 
rosrun pcl_visualization pcd_viewer transformed_$1 &
rosrun pcl_visualization pcd_viewer labeled_$1
rm  labeled_labeled_*
rm transformed_labeled_labeled_*
