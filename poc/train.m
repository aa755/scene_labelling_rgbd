function [WN we minVal Bcomb Xcomb]=train(nodeMat,edgeMat)
    [Scenes nodeFeats NL numLabels]= parseScenes(nodeMat,edgeMat,5);
    numLabels
    lenWE=size(edgeMat,2)-5;
    minVal=Inf;
                options = optimset('GradObj','off','Display','iter-detailed','LargeScale','on',...
                'MaxIter',15,'Hessian','off','TolFun',1e-6,'DerivativeCheck','off',...
                'FinDiffType','central','TolX',1e-6);
            
we=10*rand([lenWE,numLabels*numLabels]);
WN = inv(nodeFeats'*nodeFeats) * nodeFeats'*NL;
WN=WN';

numNodes=size(nodeFeats,1);
Xcomb=[];
lenWN=size(nodeFeats,2);
for i=1:numNodes
    Xt=[];
    for j=1:numLabels
        Xt=blkdiag(Xt,nodeFeats(i,:));
    end
    Xcomb=[Xcomb;Xt];
end
Yt=NL';
Ycomb=Yt(:);
%size(Xcomb)
%size(Ycomb)
%WNt = inv(Xcomb'*Xcomb) * Xcomb'*Ycomb;
%WNt=reshape(WNt,[lenWN numLabels]);
%WNt=WNt';
i=0;

while(true)
%for i=1:5
size(we)
    
                [we val]=fmincon(@(x)computeGoodness(x,WN,Scenes),we,[],[],[],[],0*ones([lenWE,numLabels*numLabels]),100*ones([lenWE,numLabels*numLabels]),[],options);
                if(val<minVal)
                    minVal=val
                    minWE=we;
                end
                i=i+1;
                WEall{i}=we;
                WNall{i}=WN;
                VALall{i}=val;
                    save('WE.mat','WNall','WEall','minVal','VALall','minWE');

                [WN val]=fminunc(@(x)computeGoodness(we,x,Scenes),WN,options);
                i=i+1;
                WEall{i}=we;
                WNall{i}=WN;
                VALall{i}=val;                
end
end