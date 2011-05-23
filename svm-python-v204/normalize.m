nodeD = load('data_nodefeats.txt');
for f = 4:size(nodeD,2)
	m = mean(nodeD(:,f));
    s = std(nodeD(:,f));
	assert (s ~= 0);
    nodeD(:,f) = (nodeD(:,f) - m)/s;
end
dlmwrite('data_nodefeats.n.txt',nodeD,'\t')

nodeE = load('data_edgefeats.txt');
for f = 6:size(nodeE,2)
	m = mean(nodeE(:,f));
    s = std(nodeE(:,f));
	assert (s ~= 0);
    nodeE(:,f) = (nodeE(:,f) - m)/s;
end
dlmwrite('data_edgefeats.n.txt',nodeE,'\t')
