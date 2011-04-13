%NF ,NL, EF,EL
function getQPCoeffs()



K=5; % number of labels
N = length (NL);% number of nodes
E = size (EN,1); % number of edges


WE = ones(K,size(EF,2),K);


M = sparse( K*K*E,N*K);

r=0;
for i = 1:E
    for l = 1:K
       for k = 1:K
           n1 = EN(i,1); % index of node 1
           n2 = EN(i,2); % index of node 2
 %          r = (i-1)*K*K + j*k;
           r=r+1;
          % EF(i,:)
           w = WE(l,:,k)* edgeR{l,k}'; %EF(i,:)';
           M(r,(n1-1)*K+l) = w;
           M(r,(n2-1)*K+k) = -w;
        end
    end    
end

I = speye(N*K,N*K);

H = I + M'*M;

theta = inv(NF'*NF) * NF'*NL
T = theta; %nodeF; %ones (K,size(NF,2));
f = zeros(1,K*N);

for i = 1:N
    for k = 1:K
        f(1,(i-1)*K+k) = T(k,:)*NF(i,:)';
    end
end



end