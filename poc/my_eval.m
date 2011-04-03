function [s,M] = my_eval (y,NL,K)

for i = 1:length(y)/K
    m = y((i-1)*K+1:i*K);
    t = max(m) * ones(K,1);
    a = find(t==m);
    if(~isempty(a))
      l(i) = a(randi([1,length(a)]));%a(1);% 
      fprintf(1,'a = %d\n',length(a));
      fprintf (1,'%d\n',l(i));
    end
    orig(i) = find(NL(i,:));
end


s = sum(l == orig)*100/length(l);

M = zeros(K,K);
for i = 1:length(l)
    M(l(i),orig(i)) = M(l(i),orig(i)) + 1;
end

end
