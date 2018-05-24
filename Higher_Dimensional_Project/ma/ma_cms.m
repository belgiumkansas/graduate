function d = ma_cms(cm1,cm2,p)
%%
%% compute Cluster Model Similarity (CMS)
%%   see Beth Logan and Ariel Salomon, ICME 2001
%%       (using Kulback Leibler Divergence and Earth Mover's Distance)
%%   and Jean-Julien Aucouturier and Francois Pachet, ISMIR 2002 + 
%%         IEEE Workshop on Model Based Processing and Coding of Audio 2002
%%       (using Gaussian Mixture Models and monte carlo sampling)
%%   for details
%%
%% USAGE
%%   frames = ma_mfcc(wav,p); %% or frames = ma_sone(wav,p);
%%   cm = ma_fc(frames,p);
%%   distance = ma_cms(cm1,cm2,p);
%%
%% note: 'KL_EMD' needs functions supplied by Yossi Rubner
%%       'monte_carlo' sampling needs netlab toolbox
%%
%%  if called without arguments, 3 test sounds are created using 
%%  "ma_test_create_wav" and the distances between these are computed
%%
%% INPUT
%%   cluster models (structure) as returned from "ma_fc" 
%%   parameter structure p
%%       p.cm_similarity = 'monte_carlo'; %% {'KL_EMD' | 'monte_carlo'}
%%       p.mc_samples = 2000; %% only needed for 'monte_carlo'
%% OUTPUT
%%   d (scalar) distance

%% elias 10.6.2004

if ~nargin,    
    disp('testing: ma_cms')
    p.mfcc.visu = 0;
    p.mfcc.fs = 11025;    
    p.mfcc.fft_size = 256;
    p.mfcc.hopsize = 128;   
    p.mfcc.num_ceps_coeffs = 20;
    p.mfcc.use_first_coeff = 0;
    
    freq = repmat([200,50,400,50,200,400,50,200],1,6);
    freq3 = repmat([200,0,400,0,200,0,50,0],1,6);
    wav = ma_test_create_wav(p.mfcc.fs,freq,.2,0.2);
    wav2 = ma_test_create_wav(p.mfcc.fs,freq.*2,.2,0.2);
    wav3 = ma_test_create_wav(p.mfcc.fs,freq3.*3,.2,0.2);
    wavA = wav3*.25 + wav2*.5 + wav + [wav(2001:end);zeros(2000,1)];
    
    [mfccA,p.DCT] = ma_mfcc(wavA,p.mfcc);
    figure;
    subplot(3,1,1)
    set(gca,'fontsize',8)
    imagesc(linspace(0,size(mfccA,2)*p.mfcc.hopsize/p.mfcc.fs,size(mfccA,2)),1:size(p.DCT,2),p.DCT'*mfccA)
    h=colorbar;
    set(h,'fontsize',8)
    set(gca,'ydir','normal','xtick',[])
    title('A) MFCC [dB-SPL]')
    %xlabel('Time [s]')
    ylabel('Frequency Bands [Mel]')
    
    freq = repmat([400,0,800,0,800,400,0,400],1,6);
    freq3 = repmat([390,0,800,0,750,0,180,0],1,6);
    wav = ma_test_create_wav(p.mfcc.fs,freq,.2,0.2);
    wav2 = ma_test_create_wav(p.mfcc.fs,freq.*2,.2,0.2);
    wav3 = ma_test_create_wav(p.mfcc.fs,freq3.*3,.2,0.2);
    wavB = wav3*.25 + wav2*.5 + wav + [wav(2001:end);zeros(2000,1)]*.5;
    
    [mfccB,p.DCT] = ma_mfcc(wavB,p.mfcc);
    
    subplot(3,1,2)
    set(gca,'fontsize',8)
    imagesc(linspace(0,size(mfccB,2)*p.mfcc.hopsize/p.mfcc.fs,size(mfccB,2)),1:size(p.DCT,2),p.DCT'*mfccB)
    h=colorbar;
    set(h,'fontsize',8)
    set(gca,'ydir','normal','xtick',[])
    title('B) MFCC [dB-SPL]')
    %xlabel('Time [s]')
    ylabel('Frequency Bands [Mel]')

    freq = repmat([400,0,400,0,400,400,50,400],1,6);
    freq3 = repmat([390,0,700,0,450,0,780,0],1,6);
    wav = ma_test_create_wav(p.mfcc.fs,freq,.2,0.2);
    wav2 = ma_test_create_wav(p.mfcc.fs,freq.*2,.2,0.2);
    wav3 = ma_test_create_wav(p.mfcc.fs,freq3.*3,.2,0.2);
    wavC = wav3 + wav2*.5 + wav*.5 + [wav(2001:end);zeros(2000,1)]*.5;
    
    [mfccC,p.DCT] = ma_mfcc(wavC,p.mfcc);
    
    subplot(3,1,3)
    set(gca,'fontsize',8)
    imagesc(linspace(0,size(mfccC,2)*p.mfcc.hopsize/p.mfcc.fs,size(mfccC,2)),1:size(p.DCT,2),p.DCT'*mfccC)
    h=colorbar;
    set(h,'fontsize',8)
    set(gca,'ydir','normal')
    title('C) MFCC [dB-SPL]')
    xlabel('Time [s]')
    ylabel('Frequency Bands [Mel]')    
    
    %sound([wavA;wavB;wavC],p.mfcc.fs)    
    
    p.cluster_type = 'kmeans';   %% {'kmeans', 'gmm'}    
    p.num_clusters = 3;
    p.covar_type   = 'diag';  %% {'spherical', 'diag', 'full'}
    
    cmA = ma_fc(mfccA,p);    
    cmB = ma_fc(mfccB,p);
    cmC = ma_fc(mfccC,p);
    
    figure;
    s=subplot(3,1,1);
    ma_cm_visu(cmA,p,s); title('A'); set(gca,'xtick',[]); xlabel('')
    s=subplot(3,1,2);
    ma_cm_visu(cmB,p,s); title('B'); set(gca,'xtick',[]); xlabel('')
    s=subplot(3,1,3);
    ma_cm_visu(cmC,p,s); title('C')

    p.cm_similarity = 'KL_EMD'; %% {'KL_EMD' | 'monte_carlo'}
    p.mc_samples = 2000;
    
    d = [];
    d(1,1) = ma_cms(cmA,cmA,p);
    d(1,2) = ma_cms(cmA,cmB,p);
    d(1,3) = ma_cms(cmA,cmC,p);
    d(2,2) = ma_cms(cmB,cmB,p);
    d(2,3) = ma_cms(cmB,cmC,p);
    d(3,3) = ma_cms(cmC,cmC,p);
    d = d+d';
    
    figure; imagesc(d); colormap gray; colorbar
    set(gca,'xtick',1:3,'xticklabel',{'A','B','C'})
    set(gca,'ytick',1:3,'yticklabel',{'A','B','C'})
    title('Distance Matrix')

    d='done';
    return
end

switch p.cm_similarity,
    case 'KL_EMD',
            supply = cm1.priors;
            demand = cm2.priors;
            if size(supply,1) > size(supply,2),
                supply = supply';
            end
            if size(demand,1) > size(demand,2),
                demand = demand';
            end
            %% sum supply == sum demand == 1
            
            %% cost is KL distance between MFCCs
            cost = zeros(length(supply),length(demand));
            for ic = 1:length(supply),
                for jc = 1:length(demand),
                    mu1 = cm1.centres(ic,:);
                    switch cm1.covar_type
                        case 'full', cov1 = diag(squeeze(cm1.covars(:,:,ic)))';
                        case 'spherical', cov1 = repmat(cm1.covars(ic),1,length(mu1));
                        case 'diag', cov1 = cm1.covars(ic,:);
                        otherwise error('MA_CMS: covar_type unknown','%s','MA_CMS: covar_type unknown');
                    end                    
                    
                    mu2 = cm2.centres(jc,:);
                    switch cm2.covar_type
                        case 'full', cov2 = diag(squeeze(cm2.covars(:,:,jc)))';
                        case 'spherical', cov2 = repmat(cm2.covars(jc),1,length(mu2));
                        case 'diag', cov2 = cm2.covars(jc,:);
                        otherwise error('MA_CMS: covar_type unknown','%s','MA_CMS: covar_type unknown');
                    end
                    
                    cost(ic,jc) = ... %% special thanks to beth logan for explaining this
                        sum(cov2./cov1 + cov1./cov2 + ...
                        (mu1-mu2).^2.*(1./cov1 + 1./cov2) - 2);
                end
            end
            d = emd_wrapper(cost,supply,demand);       
    case 'monte_carlo',
        cm1.type = 'gmm'; %% just in case it was 'kmeans' before
        cm2.type = 'gmm';
        s1 = gmmsamp(cm1, p.mc_samples);
        s2 = gmmsamp(cm2, p.mc_samples);
        p11 = sum(log(gmmprob(cm1, s1)));
        p22 = sum(log(gmmprob(cm2, s2)));
        p12 = sum(log(gmmprob(cm1, s2)));
        p21 = sum(log(gmmprob(cm2, s1)));

        d = p11 + p22 - p12 - p21;        
    otherwise error(['unknown cluster model similarity: ',p.cm_similarity]);
end