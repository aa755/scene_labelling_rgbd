#!/bin/bash
#file=$1
#cd $file
source ~/.bashrc
curDir=`pwd`
cd /opt/ros/unstable/stacks/scene_processing
make

git log | head > temp1234
git diff --color  >> temp1234
echo "@DATA###" >>temp1234
mv -f temp1234 $curDir/
cd -
cat temp1234 > data_nodefeats.txt
cat temp1234 > data_edgefeats.txt

num=0
for file in `ls transformed*.pcd`
do
  num=`expr $num + 1`
  echo $num 
  echo $file
  lfile=` echo $file | sed s/transformed_// `
  #echo $lfile
  labeledfile='labeled/'$lfile
  ls -lh $labeledfile
  bfile=`echo $lfile | sed 's/labeled_\(.*\)_segmented_xyzn.*/\1/'`
  core=`echo $lfile | sed 's/labeled_\(.*\)_segmented_xyzn.*/\1/'`
  #bagfile=`ls rgbdslam_out/*$core*`
  bagfile="rgbdslamOut/$bfile.bag.stitched.bag"
  echo  $bagfile
  ls -l $bagfile
  #mapfile="seglabels_"$num".txt"
  #echo $mapfile
  #rosrun scene_processing  compute_all_features $file $num $labeledfile $bagfile #$num_bins_color $num_bins_shape
  echo "rosrun scene_processing  compute_all_features $file $num $labeledfile $bagfile #$num_bins_color $num_bins_shape"
  
done

#mv data_feats.txt data_features_avg9feats.txt
#mv data_labels.txt data_labels_avg9feats.txt

#cd -

