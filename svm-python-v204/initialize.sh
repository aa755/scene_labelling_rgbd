mv /home/aa755/allData/data_nodefeats.txt ./
mv /home/aa755/allData/data_edgefeats.txt ./
mv /home/aa755/allData/data_scene_mapping.txt ./
perl format.pl data_nodefeats.txt data_edgefeats.txt labelmap.txt
folderName=`pwd | cut -f 7 -d '/'`
echo "folder is $folderName"

for i in `seq 1 4` 
do

sed s/data6-printer-rectified/$folderName/ fold$i/train$i >temp
mv temp fold$i/train$i

sed s/data6-printer-rectified/$folderName/ fold$i/test$i >temp
mv temp fold$i/test$i
done
