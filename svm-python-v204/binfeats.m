dataN = load('temp_data_nodefeats.txt');
dataNB = dataN(:,1:3)
numFeats=size(dataN,2)-4+1;
binStumps=zeros(10,numFeats);
for f = 4:size(dataN,2)
  featv = sort(dataN(:,f));
  binsize = floor(length(featv)/10)
  for b = 1:9
     binv(b) = featv(b*binsize)
  end
  binv(10) = featv(length(featv))
  binStumps(:,f-3)=binv;
  for b =1:10
    dataNB = [dataNB dataN(:,f)<binv(b)];
  end
end
dlmwrite('temp_data_nodefeats.b.txt',dataNB,'\t')
dlmwrite('binStumpsN.txt',binStumps','\t')

dataE = load('temp_data_edgefeats.txt');
dataEB = dataE(:,1:5)
numFeats=size(dataE,2)-6+1;
binStumps=zeros(10,numFeats);
for f = 6:size(dataE,2)
  featv = sort(dataE(:,f));
  binsize = floor(length(featv)/10)
  for b = 1:9
     binv(b) = featv(b*binsize)
  end
  binv(10) = featv(length(featv))
  binStumps(:,f-5)=binv;
  for b =1:10
    dataEB = [dataEB dataE(:,f)<binv(b)];
  end
end
dlmwrite('temp_data_edgefeats.b.txt',dataEB,'\t')
dlmwrite('binStumpsE.txt',binStumps','\t')
quit
