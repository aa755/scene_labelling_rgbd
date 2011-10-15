file=$1
num_bins_color=5
num_bins_shape=5
cd $file

num=0
for file in `ls transformed*.pcd`
do
  num=`expr $num + 1`
  echo $file,$num >> "data_scene_mapping.txt"
  
  rosrun scene_processing compute_all_features $file $num #$num_bins_color $num_bins_shape
  
done

#mv data_feats.txt data_features_avg9feats.txt
#mv data_labels.txt data_labels_avg9feats.txt

cd -

