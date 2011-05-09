for i in `seq 1 4`
do
#modelFile=`ls -rt fold$i/imodels/ | tail -1`
modelFile=model.w4.c0.1.e0.01.warm
modelFolder=fold$i/models
#ls -lh fold$i/imodels/$modelFile
mkdir fold$i/pred
../svm_python_classify --m svmstruct_mrf  fold$i/test$i $modelFolder/$modelFile fold$i/pred/pred.IP.sum1.$modelFile > fold$i/pred/out.sum1.IP.$modelFile &
#sleep 60
done    



