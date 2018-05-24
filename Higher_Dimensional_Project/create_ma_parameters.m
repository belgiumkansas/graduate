function [p] = create_ma_parameters(fs)
%%
%% create the default paramaters for ma
%%    [p] = create_ma_parameters(fs)
%%
%% USAGE
%%   p = create_ma_parameters(fs);  %% normal usage
%%
%% INPUT
%%   fs the frequency of tracks
%%                                
%% OUTPUT
%%   p struct conatains all the default parameters for ma functions
%%

%% parameters for similarity measures
    p.sone.fs = fs;
    p.sone.outerear = 'modified_terhardt';
    p.sone.fft_size = 512;
    p.sone.hopsize = 256;

    p.sh.hist_res = 25;

    p.fp.sequence.length = 512;
    p.fp.sequence.hopsize = 256;
    p.fp.sequence.windowfunction = 'boxcar';
    p.fp.fs = fs;                    
    p.fp.fft_hopsize = p.sone.hopsize; 
    p.fp.visu = 0;

    p.ph.sequence.length = 1024;
    p.ph.sequence.hopsize = 512; 
    p.ph.sequence.windowfunction = 'hann';
    p.ph.fft_hopsize = p.sone.hopsize; 
    p.ph.acc = 5;      
    p.ph.minbpm = 40;  
    p.ph.maxbpm = 240; 
    p.ph.hist_res = 30;
    p.ph.maxval = 0.4;
    p.ph.minval = 0; 
    p.ph.visu = 0;
    p.ph.preferred_tempo = 'moleants';
    p.ph.fs = fs;

    p.mfcc.visu = 0;
    p.mfcc.fs = fs;
    p.mfcc.fft_size = 512;
    p.mfcc.hopsize = 256;
    p.mfcc.num_ceps_coeffs = 21;
    p.mfcc.use_first_coeff = 0;


    p.ls.cluster_type = 'kmeans';
    p.ls.num_clusters = 30;
    p.ls.covar_type   = 'diag';  
    p.ls.cm_similarity = 'KL_EMD';

    p.ap.cluster_type = 'gmm';   
    p.ap.num_clusters = 30;
    p.ap.covar_type   = 'diag';  
    p.ap.cm_similarity = 'monte_carlo';
    p.ap.mc_samples = 2000;


end

