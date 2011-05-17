@C = (0.1);
$e= 0.01;
for $c (@C)
{
  print "C= $c\n";
  for (my $i = 1; $i<= 4; $i++)
  {
    print "\tRunning $i fold ...\n";
    chdir("fold$i");
    $pwd = `pwd`;
    print "$pwd\n";

	`git log | head > logs/log.w4.$c.e.01.sum1.warm.IPafter10`;
	`git diff >> logs/log.w4.$c.e.01.sum1.warm.IPafter10`;
    my $train = `../../svm_python_learn --m svmstruct_mrf -c $c  -e $e  train$i models/model.w4.c$c.e$e.warm>> logs/log.w4.c$c.e$e.warm & `;
#   `sleep 300`;
    chdir("../");
  # `sleep 1200`;
  }
# for ( $i = 1; $i<= 5; $i++) 
#  {
#    print "\tRunning $i fold ...\n";
#    my $accuracy = `./svm_python_classify  $d/data/test$i $d/models/model.$c.$i $d/pred/pred.$c.$i > $d/pred/out.$c.$i`;

 # }
}

