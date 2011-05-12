
for i in `seq 1 4` 
do
  mv fold$i/train$i ./
  mv fold$i/test$i ./

  rm -rf fold$i/*

  mkdir fold$i/logs
  mkdir fold$i/pred
  mkdir fold$i/imodels
  mkdir fold$i/models
  
  mv train$i fold$i/
  mv test$i fold$i/
done
