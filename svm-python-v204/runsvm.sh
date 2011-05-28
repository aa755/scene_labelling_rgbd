c=$1
e=$2
i=$3
modelFile=$4
modelFolder=$5
suffix=$6
cmethod=$7
lmethod=$8

cd fold$i
../../svm_python_learn --m svmstruct_mrf --l micro --lm $lmethod -c $c  -e $e  train$i models/$modelFile >> logs/log.$suffix 

sleep 2
../../svm_python_classify --m svmstruct_mrf --l micro --lm $lmethod --cm qbpo test$i models/$modelFile pred/pred.qbpo.$modelFile > pred/out.qbpo.$modelFile
../../svm_python_classify --m svmstruct_mrf --l micro --lm $lmethod --cm $cmethod test$i models/$modelFile pred/pred.$cmethod.$modelFile > pred/out.$cmethod.$modelFile
cd ../ 
