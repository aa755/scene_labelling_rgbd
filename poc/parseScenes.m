function [Scenes nodeFeats NL numLabels ]= parseScenes(nodeMat,edgeMat,minExamples)

    numScenes=max(nodeMat(:,1));
    Scenes=cell(numScenes,1);

    
numLabels=max(nodeMat(:,3))
count = zeros(numLabels,1);
for i = 1:size(nodeMat,1)
    if (nodeMat(i,3) ~= 8 && nodeMat(i,3) ~= 17 && nodeMat(i,3) ~= 19 && nodeMat(i,3) ~= 40&& nodeMat(i,3) ~= 15)
        count(nodeMat(i,3)) = count(nodeMat(i,3)) + 1;
    end
end

nodeMat_new = [];
edgeMat_new = [];

final_labels = find(count>minExamples)
numLabels=size(final_labels,1)

for i = 1:size(nodeMat,1)
    if(count(nodeMat(i,3)) > minExamples)
        index = find(final_labels == nodeMat(i,3));%*ones(length(final_labels),1));
        assert(index>0);
        nodeMat_new = [nodeMat_new ; nodeMat(i,:)];
        nodeMat_new(end,3)=index;

    end
end

for i = 1:size(edgeMat,1)
    if(count(edgeMat(i,4)) > minExamples && count(edgeMat(i,5)) > minExamples)
        index1 = find(final_labels == edgeMat(i,4));%*ones(length(final_labels),1));
        index2 = find(final_labels == edgeMat(i,5));%*ones(length(final_labels),1));
        edgeMat_new = [edgeMat_new ; edgeMat(i,:)];
        edgeMat_new(end,4)=index1;
        edgeMat_new(end,5)=index2;
    end
end

nodeMat=nodeMat_new;
edgeMat=edgeMat_new;


    for i=1:numScenes
        indicesNodes=find(nodeMat(:,1)==i);
        indicesEdges=find(edgeMat(:,1)==i);
        min(indicesNodes)
        max(indicesNodes)
        Scenes{i}=Scene(nodeMat(indicesNodes,:),edgeMat(indicesEdges,:),i,numLabels);
    end
            numNodes=size(nodeMat,1);
            NL=zeros(numNodes,numLabels);
            nodeFeats=nodeMat(:,4:end);
            for i=1:numNodes
                NL(i,nodeMat(i,3))=1;
            end
            
end