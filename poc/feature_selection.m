function [ac,ar] = feature_selection (X,Y,f)

X = X(:,f); 
[a,b] = run (X, Y, 25,30);
ac = a;

 
for i =1:100 ; X_rand = rand (size(X,1),size(X,2)); [a(i),b] = run (X_rand , Y, 25,30);  end
ar = mean(a);

end