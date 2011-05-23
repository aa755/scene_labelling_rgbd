dataN = load('data_nodefeats.n.txt');
dataNB = dataN(:,1:3)
for f = 4:size(dataN,2)
  featv = sort(dataN(:,f));
  binsize = floor(length(featv)/10)
  for b = 1:9
     binv(b) = featv(b*binsize)
  end
  binv(10) = featv(length(featv))
  for b =1:10
    dataNB = [dataNB dataN(:,f)<binv(b)];
  end
end
dlmwrite('data_nodefeats.b.txt',dataNB,'\t')

dataE = load('data_edgefeats.txt');
dataEB = dataE(:,1:5)
for f = 6:size(dataE,2)
  featv = sort(dataE(:,f));
  binsize = floor(length(featv)/10)
  for b = 1:9
     binv(b) = featv(b*binsize)
  end
  binv(10) = featv(length(featv))
  for b =1:10
    dataEB = [dataEB dataE(:,f)<binv(b)];
  end
end
dlmwrite('data_edgefeats.b.txt',dataEB,'\t')
