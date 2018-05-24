function my_pdf = ma_cm_visu(cm,p,s)
%%
%% my_pdf = ma_cm_visu(cm,p,s)
%%
%% visualize Cluster Model (CM)
%%   see "ma_fc" for details
%% 
%% INPUT
%%   cm (structure) as returned from "ma_fc"
%%   parameter structure p
%%       p.cluster_type = 'kmeans'; %% {'kmeans' | 'gmm'}
%%       p.DCT (matrix) as returned from ma_mfcc, 
%%                      if not given assume no DCT compression used
%%   s (handle to subplot) optional, if given no new figure is created
%%
%%   some visualization parameters can only be set in the code
%%
%% OUTPUT
%%   my_pdf (matrix) dimensions: dB vs frequency band (same as visualized)
%%
%% note: to compute distances between two CMs use "ma_cms"

%% elias 2.6.2004

if nargin==0,
    ma_fc; %% includes test call for ma_cm_visu
    my_pdf = 'done';    
    return
end

if ~isfield(p,'DCT'), 
    warning('assuming no DCT compression because p.DCT not set!');
    p.DCT = eye(cm.nin);
end

p.min_var_ratio = 500; %% set to realmax for most accurate visualization
p.mini_y = 0; %% limit dB values for y-axis if set to 0 range is determined automatically
p.maxi_y = 0;
p.resolution_x = size(p.DCT,2)*5; 
p.resolution_y = 100; 

switch cm.covar_type, %% convert to 'diag'
    case 'full', 
        for i=1:cm.ncentres,
            tmp.covars(i,:) = diag(squeeze(cm.covars(:,:,i)));
        end
        cm.covars = tmp.covars;
    case 'spherical',
        for i=1:cm.ncentres,
            tmp.covars(i,:) = repmat(cm.covars(i),cm.nin,1);
        end
        cm.covars = tmp.covars;
    case 'diag', %% do nothing    
    otherwise error(['unknown covar type: ',cm.covar_type])
end        

for i=1:cm.ncentres,
    x(i,:) = cm.centres(i,:)*p.DCT;
end
if p.mini_y==0,
    tmp.range = max(x(:))-min(x(:));
    p.mini_y = min(x(:))-tmp.range*0.1;
end
if p.maxi_y==0,
    tmp.range = max(x(:))-min(x(:));
    p.maxi_y = max(x(:))+tmp.range*0.1;
end

C = zeros(size(cm.centres,1),p.resolution_x);
for j=1:cm.ncentres,
    C(j,:) = interp1(1:size(p.DCT,2),cm.centres(j,:)*p.DCT,linspace(1,size(p.DCT,2),p.resolution_x),'cubic');
    V(j,:) = interp1(1:size(p.DCT,2),cm.covars(j,:)*p.DCT,linspace(1,size(p.DCT,2),p.resolution_x),'cubic');
    P(j) = cm.priors(j);
    x2(j,:) = interp1(1:size(p.DCT,2),x(j,:),linspace(1,size(p.DCT,2),p.resolution_x),'cubic');
end

min_var = max(abs(V(:)))/p.min_var_ratio;
V(V(:)<min_var & V(:)>=0) = min_var;
V(V(:)<min_var & V(:)<0) = min_var;

xx = linspace(p.mini_y,p.maxi_y,p.resolution_y);

for k=1:p.resolution_x%size(V,2),
    tmp_pdf = zeros(1,p.resolution_y);
    for j=1:size(cm.centres,1),
        tmp_pdf = tmp_pdf + P(j)*normpdf(xx,C(j,k),sqrt(V(j,k)));
    end
    my_pdf(k,:) = tmp_pdf;
end    

%my_pdf = imfilter(my_pdf,fspecial('gaussian'),'replicate');

if nargin<3,
    figure; 
else
    subplot(s);
end
set(gca,'fontsize',8)
imagesc(1:size(p.DCT,2),linspace(p.mini_y,p.maxi_y,p.resolution_y),my_pdf'); 
hold on
for i=1:cm.ncentres,
    plot(linspace(1,size(p.DCT,2),length(x2(i,:))),x2(i,:),'k');
end
set(gca,'ydir','normal')
colormap hot
xlabel('Frequency Bands')
ylabel('Loudness')
