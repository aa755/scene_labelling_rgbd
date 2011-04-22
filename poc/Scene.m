classdef Scene < handle
    %LEARNER A machine learning algorithm
    %   A machine learning algorithm. Can be trained and then used to make
    %   predictions
    
    properties (GetAccess=public, SetAccess=public)
        nodeFeats;
        nodeLabels;
        nodeLabelIndicators;
        nodeLableIndicatorVector;
        nodeIds;
        edgeFeats;
        edgeNodes;
        edgeNodeIndices;
        sceneIndex;
        numNodes;
        numEdges;
        numLabels;
        WN;
        WE;
    end
    
    methods
        
        function obj = Scene(nodeMat, edgeMat, sceneIndex,K)
            obj.sceneIndex=sceneIndex;
            obj.nodeIds=nodeMat(:,2);
            obj.nodeLabels=nodeMat(:,3);
            obj.nodeFeats=nodeMat(:,4:end);
            obj.edgeNodes=edgeMat(:,2:3);
            obj.edgeFeats=edgeMat(:,6:end);
            obj.numEdges=size(obj.edgeNodes,1);
            obj.numNodes=size(obj.nodeIds,1);
            for i=1:obj.numEdges
                obj.edgeNodeIndices(i,1)=find(obj.nodeIds==obj.edgeNodes(i,1));
                obj.edgeNodeIndices(i,2)=find(obj.nodeIds==obj.edgeNodes(i,2));
            end
            obj.numLabels=K; % number of labels
            obj.nodeLabelIndicators=zeros(obj.numNodes,K);
            for i=1:obj.numNodes
                obj.nodeLabelIndicators(i,obj.nodeLabels(i))=1;
                assert(sceneIndex==nodeMat(i,1));
            end
            NLtranspose=obj.nodeLabelIndicators';
            obj.nodeLableIndicatorVector=NLtranspose(:);

        end
        
        function [err]=getTrainingErrorNorm(obj,WN,WE)
            [H f]=obj.getQPCoeffs(WN,WE);
            diff=obj.nodeLableIndicatorVector-H\(f');
            err=diff'*diff; 
         %   err=sum(abs(diff)); 
            
        end
        
        %nodeFeats ,NL, edgeFeatures,EL
        function [H f]=getQPCoeffs(obj,WN,WE)
%            obj.WN=WN;
           % disp('entered')
           % tic;
            N = obj.numNodes;% number of nodes
            E = obj.numEdges; % number of edges
            
            
            %WE = ones(K,size(edgeFeatures,2),K);
            
            K=obj.numLabels;
            %M = sparse( K*K*E,N*K);
            MXindices=zeros(1,2*K*K*E);
            MYindices=zeros(1,2*K*K*E);
            MValues=zeros(1,2*K*K*E);
            r=0;
            for i = 1:E
                for l = 1:K
                    for k = 1:K
                        in1 = obj.edgeNodeIndices(i,1); % id of node 1
                        in2 = obj.edgeNodeIndices(i,2); % id of node 2
                        r=r+1;
              %          sezEd=size(obj.edgeFeats(i,:))
             % l;
             % k;
     
                        w = WE(:,l,k)'* obj.edgeFeats(i,:)';
                        index=2*r-1;
                    MXindices(index)=r;
                    MYindices(index)=(in1-1)*K+l;
                    MValues(index)=w;
                        
                        index=2*r;
                    MXindices(index)=r;
                    MYindices(index)=(in2-1)*K+k;
                    MValues(index)=-w;
                        
%                        M(r,(in1-1)*K+l) = w;
 %                       M(r,(in2-1)*K+k) = -w;
 
                    end
                end
            end
            
            M=sparse(MXindices,MYindices,MValues,K*K*E,N*K);
        
            I = speye(N*K,N*K);
            
            H =2*( I + M'*M);
            
            %theta = inv(nodeFeats'*nodeFeats) * nodeFeats'*NL
            T = WN; %nodeF; %ones (K,size(nodeFeats,2));
            f = zeros(1,K*N);
            
            for i = 1:N
                for k = 1:K
                    f(1,(i-1)*K+k) = -2*T(k,:)*obj.nodeFeats(i,:)';
                end
            end
           % toc;
            

        end
        
        function [yIndicatorsHat]=doInference(obj,WE)
           obj.WE=WE; 
           [H f]=obj.getQPCoeffs(obj.WN,obj.WE);
           yIndicatorsHat=quadprog(H,f);
        end
        
        function [norm2,percentCorrect, confMatrix]=evaluate(obj,yIndicatorsHat)
            y=yIndicatorsHat;
            norm2=norm(yIndicatorsHat-obj.nodeLableIndicatorVector);
            K=obj.numLabels;
            for i = 1:length(y)/K
                m = y((i-1)*K+1:i*K);
                t = max(m) ;%* ones(K,1);
                a = find(t==m);
                if(~isempty(a))
                    l(i) = a(randi([1,length(a)]));%a(1);%
                    % fprintf(1,'a = %d\n',length(a));
                    % fprintf (1,'%d\n',l(i));
                end
                orig(i) = obj.nodeLabels(i);
            end
            
            
            percentCorrect = sum(l == orig)*100/sum(orig<=K);
            
            M = zeros(K,K);
            for i = 1:length(l)
                M(l(i),orig(i)) = M(l(i),orig(i)) + 1;
            end
            confMatrix=M;
        end
        
    end
end
