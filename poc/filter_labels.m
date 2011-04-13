function [X_new,Y_new,final_labels]  = filter_labels (X,Y,n)

count = zeros(max(Y),1);
for i = 1:length(Y)
    if (Y(i) ~= 8 && Y(i) ~= 17 && Y(i) ~= 19)
        count(Y(i)) = count(Y(i)) + 1;
    end
end
X_new = [];
Y_new = [];

final_labels = find(count>n)
%find(max(count) == count)

for i = 1:length(Y)
    if(count(Y(i)) > n)
        index = find(final_labels == Y(i)*ones(length(final_labels),1));
        X_new = [X_new ; X(i,:)];
        Y_new = [Y_new ; index];
    end
end


end
