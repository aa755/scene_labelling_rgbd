function [X_new, Y_new ,X_test_new, Y_test_new] = preprocess (X,Y,X_test,Y_test)
 X_new = [];
 X_test_new = [];
 for i = 1:size(X,2) ; if (sum(X(:,i)) ~= 0 ) X_new = [X_new X(:,i) ] ; X_test_new = [X_test_new X_test(:,i)]; end ;end
 
 Y_new = sparse(length(Y),max(Y));
 for i = 1:length(Y) ; Y_new(i,Y(i)) = 1; end
 
 Y_test_new = sparse(length(Y_test),max(Y_test));
 for i = 1:length(Y_test) ; Y_test_new(i,Y_test(i)) = 1; end

end