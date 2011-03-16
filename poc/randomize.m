
C = randi([2,5]);
indices = randsample (size(NF,1), int8(size(NF,1)/2) );

Orig_NF = NF;

for i = 1:length(indices)

  if NF(indices(i),C) == 1
	NF(indices(i),C) = 0;
  else 
    NF(indices(i),C) = 1;
  end
end

