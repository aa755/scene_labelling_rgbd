function [WN WE val]=train(nodeMat,edgeMat)
    [Scenes WN numLabels]= parseScenes(nodeMat,edgeMat,5);
    numLabels
    lenWE=size(edgeMat,2)-5;
    minVal=Inf;
                options = optimset('GradObj','off','Display','iter-detailed','LargeScale','on',...
                'MaxIter',15,'Hessian','off','TolFun',1e-6,'DerivativeCheck','off',...
                'FinDiffType','central','TolX',1e-6);
%    while(true)
for i=1:5
    randomValue=10*rand([lenWE,numLabels*numLabels])
                [WE{i} val(i)]=fmincon(@(x)computeGoodness(x,WN',Scenes),randomValue,[],[],[],[],0*ones([lenWE,numLabels*numLabels]),100*ones([lenWE,numLabels*numLabels]),[],options);
                if(val<minVal)
                    minVal=val
                    save('WE.mat','WE','minVal','val');
                end
                minVal
end
end