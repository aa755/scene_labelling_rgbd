file=$1
num_bins_color=5
num_bins_shape=5
cd $file

for file in `ls *_binary.pcd`
do
  echo $file
  rosrun scene_processing compute_features $file $num_bins_color $num_bins_shape
  
done

mv data_feats.txt data_features_$num_bins_color_$num_bins_shape.txt
mv data_labels.txt data_labels_$num_bins_color_$num_bins_shape.txt

cd -

