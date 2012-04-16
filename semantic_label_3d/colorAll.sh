for file in `dir -d data_scene*.pcd` ; do
rosrun semantic_label_3d colorImage $file ~/scene_labelling/scene_processing/label2color_office.txt
done

mkdir labeled
mv data_*.png labeled/


for file in `dir -d data_scene*.pcd` ; do
rosrun semantic_label_3d colorImage $file ~/scene_labelling/scene_processing/label2color_office.txt NoLabels
done


mkdir unlabeled
mv data_*.png unlabeled/
