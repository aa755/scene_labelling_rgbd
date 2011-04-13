function [X_new, Y_new ,X_test_new, Y_test_new, labels] = preprocess_combined (X,Y,X_test,Y_test)

% combine test train
x_c = [X;X_test];
y_c = [Y;Y_test];

% filter the lables based on freq
[x_f,y_f,labels] = filter_labels (x_c,y_c,5);

% split into test and train
 
X=[];
Y=[];
X_test = [];
Y_test = [];

for i = 1:length(y_f)
    if(rand(1) > 0.8) 
        X_test = [X_test; x_f(i,:)];
        Y_test = [Y_test; y_f(i)];
    else
        X = [X; x_f(i,:)];
        Y = [Y; y_f(i)];
    end
end

% remove zero columns (features)
 X_new = [];
 X_test_new = [];
 for i = 1:size(X,2) ; if (sum(X(:,i)) ~= 0 ) X_new = [X_new X(:,i) ] ; X_test_new = [X_test_new X_test(:,i)]; end ;end

 % change to indicator labels 
 Y_new = sparse(length(Y),max(Y));
 for i = 1:length(Y) ; Y_new(i,Y(i)) = 1; end
 
 Y_test_new = sparse(length(Y_test),max(Y));
 for i = 1:length(Y_test) ; Y_test_new(i,Y_test(i)) = 1; end

end