#!/bin/bash
data=$1
descrip="$1 data (other info can go here)"
echo $descrip
#method=sum1.IP
loss=micro
lmethod=objassoc
cmethod=sum1.IP
objmapfile="../"$data"_objectMap.txt"
c=0.001
e=0.01
pid=(0 0 0 0)
for i in `seq 1 4`
do
#modelFile=`ls -lrt fold$i/imodels/model.c4.0.m* | cut -f 3 -d '/'| tail -1`

suffix=c$c.e$e.$lmethod
modelFile=model.$suffix

modelFolder=data/$data/fold$i/models
echo "out.$method.$modelFile" >> data/$data/fold$i/lastout.txt
sh run_svm.sh $c $e $i $modelFile $modelFolder $suffix $cmethod $lmethod $loss $objmapfile $data &
p=$!
pid[$i]=$p
#sleep 60
done 
  
ps
echo ${pid[1]},${pid[2]},${pid[3]},${pid[4]} 
wait ${pid[1]}
wait ${pid[2]}
wait ${pid[3]}
wait ${pid[4]} 
echo "processes completed!"
perl get_avg_pr.pl out.$cmethod.$modelFile data/$data/ > data/$data/avg_pr.$cmethod.$modelFile
method=$suffix.$cmethod
perl get_confusion_matrix.pl out.$cmethod.$modelFile $method data/$data/ > data/$data/confusionM.$method

rm runinfo
echo $HOSTNAME >> runinfo
pwd >> runinfo

echo "description: $descrip" >> runinfo
echo "method : $method" >> runinfo
echo "loss: $loss" >> runinfo

echo "errors:" >> runinfo
cat errfile >> runinfo
rm errfile

echo "" >> runinfo
echo "~~~~~~~~~~~~~~~" >> runinfo
echo "" >> runinfo
echo "" >> runinfo
cat data/$data/avg_pr.$cmethod.$modelFile >> runinfo
echo "" >> runinfo
echo "~~~~~~~~~~~~~~~" >> runinfo
echo "" >> runinfo
echo "" >> runinfo
cat data/$data/confusionM.$method >> runinfo

