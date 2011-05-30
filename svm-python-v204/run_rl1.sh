#!/bin/bash
descrip="homedata, nonassoc, 5 assoc feats , no volume feat , radious 0.3, "
#method=sum1.IP
loss=micro
lmethod=nonassoc
cmethod=sum1.IP
objmapfile="/critical/scene_labelling/svm-python-v204/home_objectMap.txt"
c=0.001
e=0.01
pid=(0 0 0 0)
for i in `seq 1 2`
do
#modelFile=`ls -lrt fold$i/imodels/model.c4.0.m* | cut -f 3 -d '/'| tail -1`
suffix=c$c.e$e.$lmethod
modelFile=model.$suffix
modelFolder=fold$i/models
#ls -lh fold$i/imodels/$modelFile
echo "out.$method.$modelFile" >> fold$i/lastout.txt
sh runsvm.sh $c $e $i $modelFile $modelFolder $suffix $cmethod $lmethod $loss $objmapfile &
p=$!
pid[$i]=$p
#sleep 60
done 
  
echo ${pid[1]},${pid[2]} 
wait ${pid[1]}
wait ${pid[2]}

for i in `seq 3 4`
do
#modelFile=`ls -lrt fold$i/imodels/model.c4.0.m* | cut -f 3 -d '/'| tail -1`
suffix=c$c.e$e.$lmethod
modelFile=model.$suffix
modelFolder=fold$i/models
#ls -lh fold$i/imodels/$modelFile
echo "out.$method.$modelFile" >> fold$i/lastout.txt
sh runsvm.sh $c $e $i $modelFile $modelFolder $suffix $cmethod $lmethod $loss $objmapfile &
p=$!
pid[$i]=$p
#sleep 60
done 
  
ps
echo ${pid[3]},${pid[4]} 
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

scp runinfo hema@lion.cs.cornell.edu:~/
ssh hema@lion.cs.cornell.edu "cat runinfo | mail -s "$method" hema.swetha@gmail.com"
ssh hema@lion.cs.cornell.edu "cat runinfo | mail -s "$method" aa755@cs.cornell.edu"
