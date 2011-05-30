#mv /home/aa755/allData/data_nodefeats.txt ./
#mv /home/aa755/allData/data_edgefeats.txt ./
#mv /home/aa755/allData/data_scene_mapping.txt ./
folderName=`pwd | cut -f 5 -d '/'`
echo "folder is $folderName"

for i in `seq 1 4` 
do
#rm fold$i/pred/*
#rm fold$i/logs/*
#rm fold$i/models/*
#rm fold$i/imodels/*

sed 's/opt\/ros\/unstable\/stacks/critical\/scene_labelling/' fold$i/train$i >temp
mv temp fold$i/train$i

sed 's/opt\/ros\/unstable\/stacks/critical\/scene_labelling/' fold$i/test$i >temp
mv temp fold$i/test$i
done
sed 's/opt\/ros\/unstable\/stacks/critical\/scene_labelling/' run.sh >temp
mv temp run.sh 
