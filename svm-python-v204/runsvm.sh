c=$1
e=$2
i=$3
modelFile=$4
modelFolder=$5
suffix=$6
cmethod=$7
lmethod=$8
loss=$9
objectmapfile=$10

cd fold$i
../../svm_python_learn --m svmstruct_mrf --l $loss --lm $lmethod --omf $objectmapfile -c $c  -e $e  train$i models/$modelFile >> logs/log.$suffix 2> errfile

sleep 2
../../svm_python_classify --m svmstruct_mrf --l $loss --lm $lmethod --cm qbpo --omf $objectmapfile test$i models/$modelFile pred/pred.qbpo.$modelFile > pred/out.qbpo.$modelFile 2>> errfile
../../svm_python_classify --m svmstruct_mrf --l $loss --lm $lmethod --cm $cmethod --omf $objectmapfile test$i models/$modelFile pred/pred.$cmethod.$modelFile > pred/out.$cmethod.$modelFile 2>> errfile
cd ../ 
