$d = shift; 
#$i = shift;

my %avgA = ();
@C = (50);
`mkdir $d/models`;
`mkdir $d/logs`;

for $c (@C)
{
  print "C= $c\n";
  $a = 0;
  for ( $i = 1; $i<= 4; $i++) 
  {
   `../svm_python_learn --m svmstruct_multiclass -c $c -e 0.011  $d/traindata$i $d/models/model.$c.$i > $d/logs/log.$c.$i &`;
  }
}
