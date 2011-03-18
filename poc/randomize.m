
%C = 4;%randi([2,5]);
NF=Orig_NF;
indices = randsample (size(NF,1), int8(size(NF,1)/2) );

Orig_NF = NF;

for i = 1:length(indices)

  if NF(indices(i),C) == 1
	NF(indices(i),C) = 0;
  else 
    NF(indices(i),C) = 1;
  end
end

