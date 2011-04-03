function [a] = baseline_random (x,y,x_test,y_test)

count = 0;
for i = 1:size(y_test,1)
 l = y(randi([1,length(y)],1),:);
 
 if (sum(l == y_test(i,:)) == length(l)) 
     count = count+ 1;
 end
end

a = count*100/size(y_test,1);

end