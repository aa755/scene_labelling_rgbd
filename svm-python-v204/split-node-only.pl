
for($i=1;$i<=4;$i++)
{
  open(F,"fold$i/train$i");
  open(OF,">traindata$i");
  while(<F>){
   chomp();
   open(FF,$_);
   @data = <FF>;
   chomp($data[0]);
   ($N,$E,$K) = split/ /,$data[0];
   print "$N\t$E\t$K\n";
   for ($j = 1;$j<=$N;$j++)
   {
     print OF "$K $data[$j]";
   }
   close(FF);
  }
  close(F);
  close(OF);
}

for($i=1;$i<=4;$i++)
{
  open(F,"fold$i/test$i");
  open(OF,">testdata$i");
  while(<F>){
   chomp();
   open(FF,$_);
   @data = <FF>;
   chomp($data[0]);
   ($N,$E,$K) = split/ /,$data[0];
   print "$N\t$E\t$K\n";
   for ($j = 1;$j<=$N;$j++)
   {
     print OF "$K $data[$j]";
   }
   close(FF);
  }
  close(F);
  close(OF);
}
