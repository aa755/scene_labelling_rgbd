c=$1
e=$2
i=$3
modelFile=$4
modelFolder=$5
suffix=$6
cmethod=$7
cd fold$i
../../svm_python_learn --m svmstruct_mrf -c $c  -e $e  train$i models/$modelFile >> logs/log.$suffix 

sleep 2
../../svm_python_classify --m svmstruct_mrf  test$i models/$modelFile pred/pred.$cmethod.$modelFile > pred/out.$cmethod.$modelFile
cd ../ 
