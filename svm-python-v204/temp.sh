for i in `seq 1 4` 
do
  grep "Setting"  fold$i/logs/log.w4.c4.e0.01.* | tail -1
#  grep "^prec:"  fold$i/pred/out.c.1.e005.opt.infer.sum1 
#  echo "train$i"
#  grep "22"  fold$i/train$i
#  echo "test$i"
#  grep "22"  fold$i/test$i
#  ls -lh  fold$i/logs/log.w4.0.1.e.01.sum1.warm.IPafter10


#sed s/data18-warmRestart/data6-printer-baseline/ fold$i/train$i >temp
#cp ../data6-zSquared/fold$i/train$i fold$i/
#cp ../data6-zSquared/fold$i/test$i fold$i/
#sed s/zSquared/printer-rectified/ fold$i/train$i >temp
#mv temp fold$i/train$i

#sed s/data18-warmRestart/data6-printer-rectified/ fold$i/test$i >temp
#sed s/data18/data6/ fold$i/test$i >temp
#sed s/zSquared/printer-rectified/ fold$i/test$i >temp
#mv temp fold$i/test$i

ls -rtlh fold$i/imodels/*.c4.* | tail -1

done

