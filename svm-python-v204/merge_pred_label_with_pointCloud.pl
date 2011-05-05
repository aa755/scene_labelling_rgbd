$predfile = shift;
$labelfile = shift;
$testfile = shift;
$data = shift;

%labels = ();
open(F,$labelfile);
while(<F>){
  chomp();
  ($a,$b) = split/ /,$_;
  $labels{$b} = $a;
}
close(F);

%scenemap = ();
%segmap = ();
$count = 0;
open(F,$testfile);
while(<F>){
  chomp;
  $f = $_;
  print $_."\n";
  $sn = $f;
  $sn =~ s/(.*_)(\d+)(\.txt)/\2/;
  $count ++;
  $scenemap{$count} = $sn;
  print "scene:$sn\n";
  $c = 0;
  $n = 0;
  $segnum = 1;
  open(Fl,$f);
  while(<Fl>){
    chomp();
	$line = $_;
	if($c == 0) {($n,$e) = split/\s/,$line; print "number of nodes:$n\n"; $c++;}
    else{
		if ($n>0)
		{
			$n--;
			($l,$segid,@r) = split/\s/,$line;
			$segmap{$sn}{$segnum} = $segid; 
			$segnum++;
			print "$segid\t$segnum\n"
		}
	}
    
  }
  close(Fl);
}
close(F);

%map = ();
open(F, $predfile);
$count = 0;
while(<F>){
  chomp();
  $count ++;
  (@a) = split/\s/,$_; 
  $sn = $scenemap{$count};
  
  for $p (@a)
  {
    ($segnum,$label) = split/:/,$p;
    print "$segnum,$label\n";
     $segid = $segmap{$sn}{$segnum}; 
    $map{$sn}{$segid} = $labels{$label};
    print "$count\t$sn\t$segnum\t$segid\t$label\n";
  }
}
close(F);


for $sn (keys %map){
print "writing to ./$data/seglabels_$sn.txt\n";
  open(F,">./$data/seglabels_$sn.txt" );
  for $segid (keys %{$map{$sn}})
  {
    print F "$segid\t$map{$sn}{$segid}\n";
  }
  close(F);
}
