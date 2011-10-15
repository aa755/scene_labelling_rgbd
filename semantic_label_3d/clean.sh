fname=$1
mkdir $fname
mv *data_scene_labelling_* $fname
mv labeled $fname/
mv unlabeled $fname/
mv officelabels_found.txt $fname/
mv out*.txt $fname/
mv *.png $fname/
