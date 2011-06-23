dataN = load('temp_data_nodefeats.txt');

numFeats=size(dataN,2)-3;
numFeatsBinned=numFeats*10;
dataNB=zeros(size(dataN,1),3+numFeatsBinned);
dataNB(:,1:3) = dataN(:,1:3);
binStumps=zeros(10,numFeats);
for f = 4:size(dataN,2)
  binv=getBinStumps(dataN(:,f),10);
  binStumps(:,f-3)=binv;
  
  for b =1:10
    dataNB(:,3+b+(f-4)*10) = (dataN(:,f)<=binv(b));
  end
  
end
dlmwrite('binStumpsN.txt',binStumps','\t')

dataE = load('temp_data_edgefeats.txt');
numFeats=size(dataE,2)-5;
numFeatsBinned=numFeats*10;
dataEB=zeros(size(dataE,1),5+numFeatsBinned);
dataEB(:,1:5) = dataE(:,1:5);
binStumps=zeros(10,numFeats);
for f = 6:size(dataE,2)
  binv=getBinStumps(dataE(:,f),10);
  binStumps(:,f-5)=binv;
  
  for b =1:10
    dataEB(:,5+b+(f-6)*10) =  (dataE(:,f)<=binv(b));
  end
end
dlmwrite('binStumpsE.txt',binStumps','\t')
dlmwrite('temp_data_nodefeats.b.txt',dataNB,'delimiter','\t','precision','%d')
dlmwrite('temp_data_edgefeats.b.txt',dataEB,'delimiter','\t','precision','%d')
quit
