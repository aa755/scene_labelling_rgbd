#$labelmapfile = shift;
#$labelsfile = shift;
#$outFile=`cat fold$i/lastout.txt`;
$outFile= shift;
$dir = shift;
$labelmapfile = "./$dir/labelmap.txt";
$labelsfile = '../../scene_processing/labels.txt';

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
#$emap{'1'} = '0.011';
$emap{'0.1'} = '0.01';
#$emap{'.01'} = '01';
my %avgP = ();
my %avgLP = ();
my %avgLR = ();
my %LTC = ();
my %TC = ();
my %avgR = ();
@C = ('0.1');
for $c (@C)
{
#  print "C= $c\n";
  %pr = ();
  %lpr = ();
  %tc = ();
  %ltc = ();
  %pc = ();
  %lpc = ();
  %tp = ();
  %ltp = ();
  %rc = ();
  %lrc = ();
  # read all info
#$modelFile='model.w4.c0.1.e0.01.warm';
#$outFile='model.w4.c0.1.e0.01.warm';
  for ( $i = 1; $i<= 4; $i++) 
  {
	print "outputfile=./$dir/fold$i/pred/$outFile\n";
    $ls = `ls -l ./$dir/fold$i/pred/$outFile`;
    print "$ls\n";
    $line =  `grep "^prec: " ./$dir/fold$i/pred/$outFile`   ;
    chomp ($line);

    #print "\n$line\n";
    $line =~ s/(prec:  )(.*)( recall:  )(.*)( tp:  )(.*)(  pc:  )(.*)( tc:  )(.*)/\1/; 
     #print "$2,$4,$6,$8,$10\n";
    if ($line eq ""){print "WARN: fold $i not present\n";}
    $pr{$i} = $2;
    $rc{$i} = $4;
    $tp{$i} = $6;
    $tc{$i} = $10;
    $pc{$i} = $8;
    $labellines = `grep "^label " ./$dir/fold$i/pred/$outFile`; 
    @labells = split/\n/,$labellines;
    foreach $label (@labells){
     chomp($label);
     #print "$label\n";
     $label =~ s/([^\d]*)(\d+)(  .*)/\2/;  
     #print "$label\n";
     $LabelNums{$label} =1; 
     $rest = $3;
     #print "\n$rest\n";
     $rest =~ s/(  prec:  )(.*)(  recall:  )(.*)(  tp:  )(.*)(  tc:  )(.*)(  pc:  )(.*)/\1/; 
     
     #print "$2,$4,$6,$8,$10\n";
     $lpr{$i}{$label} = $2;
     $lrc{$i}{$label} = $4;
     $ltp{$i}{$label} = $6;
     $ltc{$i}{$label} = $8;
     $lpc{$i}{$label} = $10;
     #print "$label $lpr $lrc\n"; 
    }
  }
  
  $tc_a = 0;
  $tp_a = 0;
  $pc_a = 0;
   
  # compute micro averaged precision recall 
  for  ( $i = 1; $i<= 4; $i++) 
  {
     $tc_a+= $tc{$i}; 
     $tp_a+= $tp{$i};
     $pc_a+= $pc{$i};
  }
  # 
  # precision = tp/pc
  # recall = tp/tc 
  $TC{$c} = $tc_a;
  if($pc_a !=0 ) {  $avgP{$c} =  $tp_a*100/$pc_a; } else {$avgP{$c} = 0;}
  if($tc_a !=0) { $avgR{$c} = $tp_a*100/$tc_a; } else { $avgR{$c} = 0;}
  for $l (keys %LabelNums){
    #print "$labels{$lmap{$l}}:\n";
    $tc_a = 0;
    $tp_a = 0;
    $pc_a = 0; 
    # compute micro averaged precision recall 
    for  ( $i = 1; $i<= 4; $i++) 
    {
       #print "$i\t$ltp{$i}{$l}\t$ltc{$i}{$l}\t$lpc{$i}{$l}\n";
       $tc_a+= $ltc{$i}{$l}; 
       $tp_a+= $ltp{$i}{$l};
       $pc_a+= $lpc{$i}{$l};
    }
    $LTC{$c}{$l} = $tc_a;
    print "$labels{$lmap{$l}}\t$tp_a\t$tc_a\t$pc_a\n";
    if($pc_a !=0) { $avgLP{$c}{$l} =  $tp_a*100/$pc_a; } else {$avgLP{$c}{$l} =0;}
    if($tc_a !=0) { $avgLR{$c}{$l} = $tp_a*100/$tc_a; } else {$avgLR{$c}{$l} = 0;}

  }  

}

print "Micro Averaged Precision Recall:\n";

for $c (sort {$avgP{$a} <=> $avgP{$b} }  keys %avgP)
{
  printf "%.2f\tprec: %.2f\trecall: %.2f\ttc: %d\n",$c,$avgP{$c},$avgR{$c},$TC{$c};
}
print "\n\n---------\n\n";
for $c (sort {$a<=> $b} keys %avgLP)
{
  print "C= $c\n\n";
  for $l (sort {$avgLP{$c}{$b} <=> $avgLP{$c}{$a} }  keys %{$avgLP{$c}})
  {
    printf ("%-20s\tprec: %.2f\trecall: %.2f\ttc: %d\n" , $labels{$lmap{$l}} , $avgLP{$c}{$l}, $avgLR{$c}{$l}, $LTC{$c}{$l}) ;
    #print "$l\t\t\tprec: $avgLP{$c}{$l}\trecall: $avgLR{$c}{$l}\n";
  }
  print "\n\n";
}


print "Macro Averaged Precision Recall:\n";
for $c (@C)
{
  $p = 0;
  $r = 0;
  $count = 0;
  for $l (keys %{$avgLP{$c}})
  {
     $count++;
     $p += $avgLP{$c}{$l};
     $r+= $avgLR{$c}{$l};
  }
  printf "%.2f\tprec: %.2f\trecall: %.2f\n",$c,$p/$count,$r/$count;
}

