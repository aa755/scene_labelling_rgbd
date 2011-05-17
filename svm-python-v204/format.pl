$featfile = shift;
$edgefile = shift;
$lablefile = shift;
%nodedata = ();
%edgedata = ();

#%labelmap = ('1','1','2','2','3','3','5','4','10','5','12','6');
%labelmap = ();
%invlabelmap = ();
open(F,$lablefile);
while(<F>){
  chomp;
  ($a,$b) = split/\s/,$_;
  $labelmap{$a} = $b;
  $invlabelmap{$b}  = $a;  
}
close(F);
$K = scalar keys %invlabelmap;
open(DAT, $featfile) ;
%ncount = ();
while(<DAT>)
{
  chomp();
  ($snum,$sn,$l,@feats) = split/\t/,$_;
#  delete $feats[12]; # remove the zsquare feature
  if( exists $labelmap{$l} )
  {
  $c = 0; 
  $fstring = "$labelmap{$l} $sn";
  foreach $f (@feats)
  {
     $c++;
    
     $fstring = $fstring." $c:$f"; 
  }
  $ncount{$snum} ++;
  print "$ncount{$snum}\n";
  $nodedata{$snum}{$ncount{$snum}} = $fstring;
  }
}
close(DAT);


open(DAT, $edgefile) ;
%ecount = ();
while(<DAT>)
{
  chomp();
  ($sn,$sn1,$sn2,$l1,$l2,@feats) = split/\t/,$_;
  if( exists $labelmap{$l1} && exists $labelmap{$l2} )
  {
  $c = 0; 
  $fstring = "$labelmap{$l1} $labelmap{$l2} $sn1 $sn2";
  foreach $f (@feats)
  {
     $c++;
     $fstring = $fstring." $c:$f";
  }
  $ecount{$sn} ++;
  $edgedata{$sn}{$ecount{$sn}} = $fstring;
  }
}
close(DAT);

foreach $sn (keys %ncount)
{
  $ofile = "datas_".$sn.".txt";
  open(DAT,">$ofile");
  print DAT "$ncount{$sn} $ecount{$sn} $K\n";
  foreach $l (keys %{$nodedata{$sn}} )
  {
    print DAT "$nodedata{$sn}{$l}\n";
  }
  foreach $l (keys %{$edgedata{$sn}} )
  {
    print DAT "$edgedata{$sn}{$l}\n";
  }
  close(DAT);
}



