function [s] = my_eval (y,NL,K)

for i = 1:length(y)/K
    m = y((i-1)*K+1:i*K);
    t = max(m) * ones(K,1);
    a = find(t==m);
    if(~isempty(a))
      l(i) = a(1);% a(randi([1,length(a)]));
      %l(i)
    end
    orig(i) = find(NL(i,:));
end


s = sum(l == orig)*100/length(l);

end
