function [X_new, Y_new] = preprocess (X,Y)

 for i = 1:size(X,2) ; if (sum(X(:,i)) ~= 0 ) X_new = [X_new X(:,i) ] ; end ;end
 Y_new = sparse(length(Y_t),max(Y_t));
 for i = 1:length(Y) ; Y_new(i,Y(i)) = 1; end

end