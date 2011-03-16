% labels : Wall, Monitor, Table, Chair, Ground
% % Node Features : IsWall? , IsMonitor?, IsTable? , IsChair? , IsGround?
% Relations: On-top-of , near-by, coplannar, connected
% R(Monitor, Table ) = [1,0,0,1]
% R(Chair, Ground) = [1,0,0,1]
% R(Chair, Table) = [0,1,0,0]
% R(Table, Ground) = [1,0,0,1]
% R(Wall, Monitor) = [0,1,1,0] //, [0,1,0,0]
% R(Wall, Wall) = [0,1,1,1]//, [0,1,0,1]
% R(Wall,Table) = [0,1,0,0]
% R(Wall,Ground) = [1,0,0,1]


clear;

label{1} = 'Wall'

nodeF(1,:) = [1,0,0,0,0];
nodeF(2,:) = [0,1,0,0,0];
nodeF(3,:) = [0,0,1,0,0];
nodeF(4,:) = [0,0,0,1,0];
nodeF(5,:) = [0,0,0,0,1];



edgeR{1,1} = [0,1,1,1];
edgeR{1,2} = [0,1,1,0];
edgeR{1,3} = [0,1,0,0];
edgeR{1,4} = [0,0,0,0];
edgeR{1,5} = [1,0,0,1];
edgeR{2,1} = edgeR{1,2};
edgeR{2,2} = [0,0,0,0];
edgeR{2,3} = [1,0,0,1];
edgeR{2,4} = [0,0,0,0];
edgeR{2,5} = [0,0,0,0];
edgeR{3,1} = edgeR{1,3};
edgeR{3,2} = edgeR{2,3};
edgeR{3,3} = [0,0,0,0];
edgeR{3,4} = [0,1,0,0];
edgeR{3,5} = [1,0,0,1];
edgeR{4,1} = edgeR{1,4};
edgeR{4,2} = edgeR{2,4};
edgeR{4,3} = edgeR{3,4};
edgeR{4,4} = [0,0,0,0];
edgeR{4,5} = [1,0,0,1];
edgeR{5,1} = edgeR{1,5};
edgeR{5,2} = edgeR{2,5};
edgeR{5,3} = edgeR{3,5};
edgeR{5,4} = edgeR{4,5};
edgeR{5,5} = [0,0,0,0];





% for each scene
% generate one/two tables, one/two chairs, one wall, one ground, one-three
% monitors


NLt={};



for s = 1:5
    c  = 0;
    % wall 
    l = [1 0 0 0 0];
    t = randi([1,1],1);
    for i = 1:t
      c = c+1;
      NFt{s,c} = nodeF(find(l),:); 
      NLt{s,c} = l;
    end
    % monitor 
    l = [0 1 0 0 0];
    t = randi([1,3],1);
    for i = 1:t
      c = c+1;
      NFt{s,c} = nodeF(find(l),:);
      NLt{s,c} = l;
    end
    % table 
    l = [0 0 1 0 0];
    t = randi([1,2],1);
    for i = 1:t
      c = c+1;
      NFt{s,c} = nodeF(find(l),:); 
      NLt{s,c} = l;
    end
        % chair 
    l = [0 0 0 1 0];
    t = randi([1,3],1);
    for i = 1:t
      c = c+1;
      NFt{s,c} = nodeF(find(l),:); 
      NLt{s,c} = l;
    end
        % ground 
    l = [0 0 0 0 1];
    t = randi([1,1],1);
    for i = 1:t
      c = c+1;
      NFt{s,c} = nodeF(find(l),:);
      NLt{s,c} = l;
    end
end


c=0;
scene=[0];
for i = 1:size(NFt,1)
    for j = 1:size(NFt,2)
        if(~isempty(NFt{i,j}))
            c=c+1;
            NF(c,:) = NFt{i,j};
            NL(c,:) = NLt{i,j};
        end
    end
    scene(i+1) = c;
end

c=0;
for i = 1:size(NFt,1)
    for j = 1:size(NFt,2)
        for k = 1:size(NFt,2)
        if(j~=k)
           % if ( length(NFt{i,j}) ~=0  && lenght(NFt{i,k})~=0 )
                l1 = find(NFt{i,j});
                l2 = find(NFt{i,k});
            if (( ~isempty(l1) ) && (~isempty(l2) )  )   
                
                EFt{s,j,k} = edgeR{l1,l2};
                ELt{s,j,k} = [l1,l2];
                % disp (~isempty(edgeR{l1,l2}))
                if ( sum(edgeR{l1,l2}) ~=0)
                   c = c+1;
                   EF(c,:)  = edgeR{l1,l2};
                   EN(c,:) = [scene(i)+j,scene(i)+k];
                
                end
            end
        end
        end
    end
end
