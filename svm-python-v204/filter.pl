$featfile = shift;
$tempfile = "temp_$featfile";
$headerfile = "header_$featfile";
open(HEAD,">$headerfile");
open(TEMP,">$tempfile");
open(DAT, $featfile) ;
$flag = 0;
$flag2 = 0;
while(<DAT>){  
chomp();
  if($_ =~ m/\@DATA/) { $flag = 1; $flag2 = 1;print HEAD $_."\n"; next; }
  if($flag == 0) {print HEAD $_."\n"; next;}  
  if($flag2 == 1 and $_ =~ m/^\#/) {print HEAD $_."\n";  next;}
  $flag2 = 0;
  if($_ =~ m/^\#/) {  next;}
  print TEMP "$_\n";
}
close(DAT);
close(HEAD);
close(TEMP);
