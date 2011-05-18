@C = (2);
$e= 0.01;
$method='qbpo';
for $c (@C)
{
  print "C= $c\n";
  for (my $i = 1; $i<= 4; $i++)
  {
    print "\tRunning $i fold ...\n";
    chdir("fold$i");
    $pwd = `pwd`;
    print "$pwd\n";
	$suffix="w4.c$c.e$e.$method";

	`git log | head > logs/log.$suffix`;
	`git diff >> logs/log.$suffix`;
    my $train = `../../svm_python_learn --m svmstruct_mrf -c $c  -e $e  train$i models/model.$suffix>> logs/log.$suffix & `;
   `sleep 300`;
    chdir("../");
  # `sleep 1200`;
  }
# for ( $i = 1; $i<= 5; $i++) 
#  {
#    print "\tRunning $i fold ...\n";
#    my $accuracy = `./svm_python_classify  $d/data/test$i $d/models/model.$c.$i $d/pred/pred.$c.$i > $d/pred/out.$c.$i`;

 # }
}

