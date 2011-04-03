function [a , b] = run(X , Y, K)

theta = inv(X'*X) * X'*Y;
y_pred = X*theta;
y_pred_vec = y_pred';
y_pred_vec = y_pred_vec(:);
[a,b]  = my_eval(y_pred_vec, Y,K);

end