$labelmapfile = shift;
$labelsfile = shift;
$method = shift;

%lmap=();
open(F,$labelmapfile);
while(<F>)
{
  chomp();
  ($a,$b) = split/ /,$_;
  $lmap{$b} = $a; 
}
close (F);

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
$emap{'.1'} = '005';
$emap{'.01'} = '01';
my %avgP = ();
my %avgR = ();
@C = ('.1','.01');
for $c (@C)
{
  print "C= $c\n";
  $p = 0;
  %lp = ();
  $r = 0;
  %lr = ();
  for ( $i = 1; $i<= 4; $i++) 
  {
    $pr =  `grep "^prec: " fold$i/pred/out.c$c.e$emap{$c}.$method`   ;
    chomp($pr);
    $pr =~ s/([^\d]*)([01].\d+)(.*)/\2/; 
    $rc = $3;
    $rc =~ s/([^\d]*)([01].\d+)(.*)/\2/; 
    $p+= $pr;
    $r+= $rc;
    $labellines = `grep "^label " fold$i/pred/out.c$c.e$emap{$c}.$method `; 
    @labells = split/\n/,$labellines;
    foreach $label (@labells){
     chomp($label);
     #print "$label\n";
     $label =~ s/([^\d]*)(\d+)( .*)/\2/;    
     $lpr = $3;
     $lpr =~ s/([^\d]*)([01].\d+)(.*)/\2/; 
     $lrc = $3;
     $lrc =~ s/([^\d]*)([01].\d+)(.*)/\2/; 
     $lp{$label}+= $lpr;
     $lr{$label}+= $lrc;
     #print "$label $lpr $lrc\n"; 
    }
  }
  $avgP{$c} = $p/4;
  $avgR{$c} = $r/4;
  for $l (keys %lp)
  {
    #print "$l\t$lmap{$l}\t$labels{$lmap{$l}}\n";
     $avgLP{$c}{$labels{$lmap{$l}}} = $lp{$l}/4;
     $avgLR{$c}{$labels{$lmap{$l}}} = $lr{$l}/4;
  }
}



for $c (sort {$avgP{$a} <=> $avgP{$b} }  keys %avgP)
{
  print "$c\t$avgP{$c}\t$avgR{$c}\n";
}
print "\n\n---------\n\n";
for $c (sort {$a<=> $b} keys %avgLP)
{
   print "C= $c\n\n";
  for $l (sort {$avgLP{$c}{$b} <=> $avgLP{$c}{$a} }  keys %{$avgLP{$c}})
  {
    printf ("%-20s\tprec: %.4f\trecall: %.4f\n" ,$l, $avgLP{$c}{$l}, $avgLR{$c}{$l});
    #print "$l\t\t\tprec: $avgLP{$c}{$l}\trecall: $avgLR{$c}{$l}\n";
  }
  print "\n\n";
}
