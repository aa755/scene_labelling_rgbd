for i in `seq 1 4`
do
modelFile=`ls -rt fold$i/imodels/ | tail -1`
ls -lh fold$i/imodels/$modelFile
mkdir fold$i/pred
../svm_python_classify --m svmstruct_mrf  fold$i/test$i fold$i/imodels/$modelFile fold$i/pred/pred.IP.sum1.$modelFile > fold$i/pred/out.sum1.IP.$modelFile &
#sleep 60
done    



