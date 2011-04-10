classdef Scene < handle
    %LEARNER A machine learning algorithm
    %   A machine learning algorithm. Can be trained and then used to make
    %   predictions
    
    properties (GetAccess=public, SetAccess=protected)
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
        K;
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
        
        %nodeFeats ,NL, edgeFeatures,EL
        function [H f]=getQPCoeffs(obj,WN,WE)
            
            
            
            N = obj.numNodes;% number of nodes
            E = obj.numEdges; % number of edges
            
            
            %WE = ones(K,size(edgeFeatures,2),K);
            
            K=obj.numLabels;
            M = sparse( K*K*E,N*K);
            
            r=0;
            for i = 1:E
                for l = 1:K
                    for k = 1:K
                        in1 = obj.edgeNodeIndices(i,1); % id of node 1
                        in2 = obj.edgeNodeIndices(i,2); % id of node 2
                        r=r+1;
                        sizWE=size(WE);
              %          sezEd=size(obj.edgeFeats(i,:))
              l;
              k;
                        w = WE(:,l,k)'* obj.edgeFeats(i,:)';
                        M(r,(in1-1)*K+l) = w;
                        M(r,(in2-1)*K+k) = -w;
                    end
                end
            end
            
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
            
            
            
        end
        

    end
end
