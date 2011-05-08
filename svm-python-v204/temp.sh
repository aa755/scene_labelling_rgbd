for i in `seq 1 4` 
do
#  grep "Setting"  fold$i/logs/log.w4.0.1.e.01.sum1.warm.IPafter10
#  echo "train$i"
#  grep "22"  fold$i/train$i
#  echo "test$i"
#  grep "22"  fold$i/test$i
#  ls -lh  fold$i/logs/log.w4.0.1.e.01.sum1.warm.IPafter10
#  rm  fold$i/logs/*
#  rm  fold$i/pred/*
#  rm  fold$i/imodels/*
sed s/data18-warmRestart/data6-printer-baseline/ fold$i/train$i >temp
mv temp fold$i/train$i

sed s/data18-warmRestart/data6-printer-baseline/ fold$i/test$i >temp
mv temp fold$i/test$i


done

