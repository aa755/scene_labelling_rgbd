#mv /home/aa755/allData/data_nodefeats.txt ./
#mv /home/aa755/allData/data_edgefeats.txt ./
#mv /home/aa755/allData/data_scene_mapping.txt ./
rm data_nodefeats.*.txt data_edgefeats.*.txt
rm datas_*.txt
perl filter.pl data_nodefeats.txt
perl filter.pl data_edgefeats.txt
#cp data_nodefeats.txt temp_data_nodefeats.txt
#cp data_edgefeats.txt temp_data_edgefeats.txt
#nohup matlab -nodesktop -nosplash -r normalize
nohup matlab -nodesktop -nosplash -r binfeats
#cat header_data_nodefeats.txt temp_data_nodefeats.n.txt > data_nodefeats.n.txt
#cat header_data_edgefeats.txt temp_data_edgefeats.n.txt > data_edgefeats.n.txt
cat header_data_nodefeats.txt temp_data_nodefeats.b.txt > data_nodefeats.b.txt
cat header_data_edgefeats.txt temp_data_edgefeats.b.txt > data_edgefeats.b.txt
rm temp_data_* 
rm header_data_*
perl format.pl data_nodefeats.b.txt data_edgefeats.b.txt labelmap.txt 1 
#folderName=`pwd | cut -f 7 -d '/'`
#echo "folder is $folderName"

for i in `seq 1 4` 
do

mkdir fold$i
mkdir fold$i/pred/
mkdir fold$i/logs/
mkdir fold$i/models/
mkdir fold$i/imodels/

#sed s/officedata/$folderName/ fold$i/train$i >temp
#mv temp fold$i/train$i
#sed s/officedata/$folderName/ fold$i/test$i >temp
#mv temp fold$i/test$i
done
