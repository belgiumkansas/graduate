%% simple script to create islands of music
%%
%% assume that either a matrix D is given which is either a 
%% distance matrix (size n x n, computed using e.g. "ma_cms")
%% or a data matrix of size (n x d) where d is the dimensionality
%% of the data (computed using e.g. "ma_sh")
%%
%% assume labels given as cell of strings, e.g.,
%% labels = {'song1', 'song2', 'song3'};

MSIZE = [4 6]; %% size of the SOM

D = randn(100,5);
for i=1:100,
    labels{i} = num2str(i);
end

if 1, %% do PCA compression (warning, dont try this if size(D,2)>5000
    EIGS = 2; %% number of eigenvectors to use
    mD = mean(D);
    Dm = D - repmat(mD,size(D,1),1);
    [eVts eVls] = eig(cov(Dm));
    D_orig = D;
    D = D*eVts(end-EIGS+1:end,:)';
end

sData = som_data_struct(D,'labels',labels');
sMap = som_map_struct(size(D,2),'msize',MSIZE,'sheet','rect');
sMap = som_randinit(D,sMap);

%% the radius should be optimized depending on map size, available CPU time, ...
sMap = som_batchtrain(sMap,D,'radius',linspace(max(MSIZE),0.5,500));

sMap = som_autolabel(sMap,sData);

%% visualize
S = sdh_calculate(sData,sMap);
sdh_visualize(S,'labels',sMap.labels,'fontsize',8,'fontcolor','r','sofn',0)
