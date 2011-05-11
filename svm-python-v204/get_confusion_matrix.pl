$labelmapfile = 'labelmap.txt';
$labelsfile = '/opt/ros/unstable/stacks/scene_processing/labels.txt';
$method = shift;
$numClasses=6;

%lmap=();
open(F,$labelmapfile);
while(<F>)
{
  chomp();
  ($a,$b) = split/ /,$_;
  $lmap{$b} = $a; 
}
close (F);
%LabelNums = ();
%labels= ();
$count  =0;
open(F,$labelsfile);
while(<F>)
{
  chomp();
  $count++;
  $labels{$count} = $_; 
}
close (F);


$d = shift; 
#$i = shift;
$emap{'1'} = '0.011';
$emap{'0.1'} = '0.005';
$emap{'0.01'} = '0.01';
my %avgP = ();
my %avgLP = ();
my %avgLR = ();
my %LTC = ();
my %TC = ();
my %avgR = ();
@C = ('1');
$modelFile='model.w4.c0.1.e0.01.warm';
for $c (@C)
{
  print "C= $c\n";
  # read all info
  %mat = ();  
  %m = ();
  for ( $i = 1; $i<= 4; $i++) 
  {
    $flag = 0; 
    $count = 0;
    open(F,"fold$i/pred/out.$method.$modelFile")   ;
    while(<F>){
      chomp ;
      $line = $_;
      if($line =~ m/confusion matrix:/){
#       print "\n$line\n";
       $flag = 1;
       $count = 1;
       next;
      }
      if($flag ==1 && $count <=$numClasses )
      {
        $line =~ s/([ \[]*)([^\]]*)(\]*)/\2/;
 #       print $line."\n";
        @{$mat{$c}{$i}{$count}} = split/\s+/,$line;
  #      print join(",",@{$mat{$c}{$i}{$count}})."\n";
        $count++;
      }
    }
  }

  for($i =1 ; $i<= 4; $i++)
  {
     for ( $k = 1; $k<= $numClasses; $k++) 
     {
       for ( $l = 0; $l< $numClasses; $l++) 
       { 
         print "$i\t$k\t$l\t$mat{$c}{$i}{$k}[$l]\t$m{$c}{$k}{$l+1}\n";
         $m{$c}{$k}{$l+1} += $mat{$c}{$i}{$k}[$l];
       }
     }
  }
   

}



for $c (@C)
{
  open(F,">confusionM_c$c\_$method.csv");
  @ll = ();
  print $c."\n\n";
  printf "%-16s", $blank;
  for ( $k = 1; $k<= $numClasses; $k++) {
    printf "%-5d\t",  $k ; 
    push(@ll,$labels{$lmap{$k}});
  }
  print F join(",",@ll)."\n";

  print "\n\n";
  for ( $k = 1; $k<= $numClasses; $k++)
  {
    @r = ();
    printf "%d:%-12s",  $k,$labels{$lmap{$k}} ; 
    for ( $l = 1; $l<= $numClasses; $l++) 
    {
      printf "\t%-5s",$m{$c}{$k}{$l};
      push(@r,$m{$c}{$k}{$l})
    }
    print F join(",",@r)."\n";
    print "\n";
  }
  close(F);
}


