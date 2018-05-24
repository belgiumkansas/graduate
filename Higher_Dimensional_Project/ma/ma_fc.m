function cm = ma_fc(frames,p)
%%
%% compute Frame-based Clustering (FC)
%%   see Beth Logan and Ariel Salomon, ICME'01
%%   and Jean-Julien Aucouturier and Francois Pachet, ISMIR'02
%%   for details
%%
%% frames = ma_mfcc(wav,p); %% or frames = ma_sone(wav,p);
%% cm = ma_fc(frames,p);
%% distance = ma_cms(cm1,cm2,p);
%%
%%  specific parameters for k-means and gmm need to be set in the code.
%% 
%% INPUT
%%   frames (matrix) as returned from "ma_sone" or "ma_mfcc" 
%%                   (size: frequency-bands x time)
%%   parameter structure p
%%       p.cluster_type = 'kmeans'; %% {'kmeans' | 'gmm'} 
%%                                  %% 'gmm' requires NETLAB toolbox
%%       p.num_clusters = 3;        %% number of clusters
%%       p.covar_type   = 'diag';   %% {'spherical' | 'diag' | 'full'}
%%
%% OUTPUT
%%   cm (structure) cluster model (netlab gmm model structure)
%%          type: 'gmm'
%%           nin: 19
%%      ncentres: 25
%%    covar_type: 'full'
%%        priors: [1x25 double]
%%       centres: [25x19 double]
%%        covars: [19x19x25 double]
%%          nwts: 9525
%%
%% note: to compute distances between two CMs use "ma_cms"
%%       to visualize a CM use "ma_cm_visu"

%% elias 2.6.2004
%% fixed bug reported by Mark T. Godfrey (17 Mar 2007)

if nargin==0, %% test mode
    disp('testing: ma_fc')
    
    p.mfcc.visu = 0;
    p.mfcc.fs = 11025;    
    p.mfcc.fft_size = 256;
    p.mfcc.hopsize = 128;   
    p.mfcc.num_ceps_coeffs = 20;
    p.mfcc.use_first_coeff = 1;
    
    freq = repmat([200,50,400,50,200,400,50,200],1,6);
    freq3 = repmat([200,0,400,0,200,0,50,0],1,6);
    wav = ma_test_create_wav(p.mfcc.fs,freq,.2,0.2);
    wav2 = ma_test_create_wav(p.mfcc.fs,freq.*2,.2,0.2);
    wav3 = ma_test_create_wav(p.mfcc.fs,freq3.*3,.2,0.2);
    wav = wav3*.25 + wav2*.5 + wav + [wav(2001:end);zeros(2000,1)];
    
    sound(wav,p.mfcc.fs);

    [mfcc,p.DCT] = ma_mfcc(wav,p.mfcc); %% need p.DCT if p.visu
    
    p.cluster_type = 'gmm';   %% {'kmeans', 'gmm'}    
    p.num_clusters = 3;
    p.covar_type   = 'diag';  %% {'spherical', 'diag', 'full'}
    
    cm = ma_fc(mfcc,p);
    
    figure;
    subplot(2,1,1)
    set(gca,'fontsize',8)
    imagesc(linspace(0,size(mfcc,2)*p.mfcc.hopsize/p.mfcc.fs,size(mfcc,2)),1:size(p.DCT,2),p.DCT'*mfcc)
    h=colorbar;
    set(h,'fontsize',8)
    set(gca,'ydir','normal')
    title('MFCC [dB-SPL]')
    xlabel('Time [s]')
    ylabel('Frequency Bands [Mel]')
    
    s=subplot(2,1,2);
    ma_cm_visu(cm,p,s);
    title('Cluster Model [accumulated probability]')
    xlabel('Frequency Bands [Mel]')
    ylabel('Loudness [dB-SPL]')    
    
    cm = 'done';
    return    
end

switch p.cluster_type
    case 'kmeans',
        qe_best = realmax;
        C_best = [];
        N_best = [];
        W_best = [];
        for j=1:5, %% try 5 different k-means initializations with each 20 iterations
            [C, qe, N, W] = ma_kmeans(frames',20,p.num_clusters);
            if qe < qe_best,
                qe_best = qe;
                C_best = C;
                N_best = N;
                W_best = W; %% winner aka bmus
            end
        end
        
        cm.type = 'kmeans';
        cm.nin = size(frames,1);
        cm.ncentres = p.num_clusters;
        cm.covar_type = p.covar_type;
        cm.priors = N_best'./sum(N_best);
        cm.centres = C_best;        
        for i=1:length(N_best),
            idx = find(W_best==i); %% fixed bug reported by Mark T. Godfrey (17 Mar 2007)
            if isempty(idx),
                error('shouldnt happen: cluster collapsed, no items assigned?!');
            end
            switch p.covar_type,
                case 'full',  cm.covars(:,:,i) = cov(frames(:,idx)');
                case 'diag',  cm.covars(i,:) = diag(cov(frames(:,idx)'));
                case 'spherical',  cm.covars(i) = mean(diag(cov(frames(:,idx)')));
                otherwise error(['unknown p.covar_type: ',p.covar_type])
            end
        end
        cm.nwts = prod(size(cm.covars))+prod(size(cm.centres))+prod(size(cm.priors));
        
    case 'gmm',
        p.gmmem_options = zeros(14,1);
        p.gmmem_options(1) = -1;            %% no error logs
        p.gmmem_options(3) = eps*100;       %% threshold improvment for termination
        p.gmmem_options(5) = 1;             %% reset collapsed clusters
        p.gmmem_options(14) = 30;           %% max number of iterations
        
        ddim = size(frames,1);
        cm = gmm(ddim, p.num_clusters, p.covar_type);
        cm = gmminit(cm, frames', p.gmmem_options);
        cm = gmmem(cm, frames', p.gmmem_options);
        
    otherwise error(['unkown p.cluster_type: ',p.cluster_type])
end
