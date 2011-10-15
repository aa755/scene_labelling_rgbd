#!/bin/bash
descrip="info about the data"
#method=sum1.IP
loss=micro
lmethod=objassoc
cmethod=sumLE1.IP #sum1.IP
objmapfile="/opt/ros/unstable/stacks/svm-python-v204/home_objectMap.txt"
c=0.001
e=0.01
pid=(0 0 0 0)
for i in `seq 1 4`
do
#modelFile=`ls -lrt fold$i/imodels/model.c4.0.m* | cut -f 3 -d '/'| tail -1`

suffix=c$c.e$e.$lmethod
modelFile=model.$suffix

modelFolder=fold$i/models
#ls -lh fold$i/imodels/$modelFile
echo "out.$method.$modelFile" >> fold$i/lastout.txt
#mkdir fold$i/logs
#mkdir fold$i/models
#mkdir fold$i/imodels
#mkdir fold$i/pred
sh runsvm.sh $c $e $i $modelFile $modelFolder $suffix $cmethod $lmethod $loss $objmapfile &
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
perl ../get_avg_pr.pl out.$cmethod.$modelFile > avg_pr.$cmethod.$modelFile
method=$suffix.$cmethod
perl ../get_confusion_matrix.pl out.$cmethod.$modelFile $method  > confusionM.$method

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
cat avg_pr.$cmethod.$modelFile >> runinfo
echo "" >> runinfo
echo "~~~~~~~~~~~~~~~" >> runinfo
echo "" >> runinfo
echo "" >> runinfo
cat confusionM.$method >> runinfo

scp runinfo lion.cs.cornell.edu:~/
ssh lion.cs.cornell.edu "cat runinfo | mail -s "$method" hema.swetha@gmail.com"
ssh lion.cs.cornell.edu "cat runinfo | mail -s "$method" aa755@cs.cornell.edu"
