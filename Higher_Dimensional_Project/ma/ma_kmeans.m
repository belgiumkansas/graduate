function [C,Qe,N,W,Q] = ma_kmeans(X, iter, c, C)
%%
%% simple batch k-means
%%
%% [C,Qe,N,W,Q] = ma_kmeans(X, iter, c, C)
%%
%% INPUT
%%   X    .. Data
%%   iter .. Iterations (if set to 0, only initialize)
%%   c    .. number of clusters
%%   C    .. initial Codebooks (default = randperm select k)
%% 
%% OUTPUT
%%   C    .. centers
%%   Qe   .. final quantisation error
%%   N    .. number of items mapped to each cluster
%%   W    .. winner (best matching cluster for each item)
%%   Q    .. quantisation error of each item (Qe = mean(Q))
%% 
%% used by ma_fc (frame clustering)

%% elias 2.6.2004

if ~nargin,
    disp('testing: ma_kmeans')
    %% use 2-dim test data with 5 gaussian clusters
    X = randn(30,2)/5;
    X(31:60,:) = randn(30,2)/5+repmat([-0.5 1],30,1);
    X(61:90,:) = randn(30,2)/5+repmat([1 0.5],30,1);
    X(91:100,:) = randn(10,2)/3+repmat([1.5 2.5],10,1);
    X(101:110,:) = randn(10,2)/3+repmat([-.5 2.5],10,1);
    figure; 
    
    colors = {'ro','mo','go','co','yo'};
    for iter = 0:3,
        if iter>0,
            [C, Qe, N, W] = ma_kmeans(X,1,5,C);
        else %% init
            [C, Qe, N, W] = ma_kmeans(X,0,5);
        end
        subplot(2,2,iter+1); set(gca,'fontsize',8)
        for i=1:max(W),
            plot(X(W==i,1),X(W==i,2),colors{i}); hold on
        end
        for i=1:5,
            plot(C(i,1),C(i,2),'+k')
        end
        set(gca,'xtick',[],'ytick',[])
        title(['Iteration = ',num2str(iter)])
    end
    
    C = 'done';
    return
end

[n, d] = size(X);

%% Assign first ncentres (permuted) data points as centres
if nargin<4,
    perm = randperm(n);
    C = X(perm(1:c), :);
end

X2 = 2*X';

i = 1; 
while i<=iter 
    i = i+1;
    [Q W] = min(repmat(sum(C.^2,2),1,n)-C*X2); %% quantisation error, winner (unit with smallest to data item)
    P = sparse(W,1:n,ones(n,1),c,n);           %% partition P(i,j)=1 if unit i is activated by dataitem j else P(i,j)=0
    S = P*X;                                   %% sum of mapped winners
    A = sum(P,2);                              %% activation normalization factor
    nz = find(A > 0);                          %% update only units with activation > 0
    if nz<c,
        z = find(A==0);
        perm = randperm(n);
        C(z,:) = X(perm(1:length(z)),:);
    end
    C(nz,:) = S(nz,:) ./ repmat(A(nz),1,d); 
    
    %fprintf('%2d: mqerr: %f\n',i,mean(Q));
end

[Q W] = min(repmat(sum(C.^2,2),1,n)-C*X2); %% quantisation error, winner (unit with smallest to data item)
P = sparse(W,1:n,ones(n,1),c,n);           %% partition P(i,j)=1 if unit i is activated by dataitem j else P(i,j)=0
N = full(sum(P,2));                        %% number of mapped units

Qe = mean(Q);
