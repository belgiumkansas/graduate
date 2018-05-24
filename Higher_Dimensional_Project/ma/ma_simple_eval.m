%% evaluate similarity measures
%%
%% this is a script (not a function)
%% edit it to fit your needs
%%
%% basic idea:
%%   given a directory in which there are several subdirectories
%%   each of this subdirectories contains "similar" wav files
%%   at the end the R-precission is computed

%% updated on 09-Apr-2007 (added more text output)

%% root directory in which subdirectories are located
rootdir = 'C:\Users\Johnny_Five\Music Data\tracks\';
%%rootdir = 'C:\Documents and Settings\Elias\Desktop\ma\ma\ma\test\';

p.fs = 11025; %% sample rate of all wav files in the directories

%% scandir for directories
d = dir(rootdir);
d = char(d.name);
d = d(3:end,:); %% ignore "." and ".."

dirs = {};


for i=1:size(d,1),
    dirs{i} = strcat(d(i,:));
end

data.list_wavs = {};
data.class = []; %% nummeric
data.class_name = {};
for i=1:length(dirs),
    d = dir([rootdir,dirs{i},'/*.wav']);
    d = char(d.name);
    for j=1:size(d,1),
        data.list_wavs{end+1} = [rootdir,dirs{i},'/',strcat(d(j,:))];
        data.class(end+1) = i;
    end
    data.class_name{end+1} = [rootdir,dirs{i}];
end

disp(['found ',num2str(length(data.list_wavs)), ...
    ' wav files in ',num2str(length(data.class_name)),' directories.']);

if length(data.list_wavs)<2 || length(data.class_name)<2,
    error('not enough data for evaluation.')
end

%% parameters for similarity measures
p.sone.fs = p.fs;
p.sone.outerear = 'modified_terhardt';
p.sone.fft_size = 512;
p.sone.hopsize = 256;

p.sh.hist_res = 25;

p.fp.sequence.length = 512;
p.fp.sequence.hopsize = 256;
p.fp.sequence.windowfunction = 'boxcar';
p.fp.fs = p.fs;                    
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
p.ph.fs = p.fs;

p.mfcc.visu = 0;
p.mfcc.fs = p.fs;    
p.mfcc.fft_size = 512;
p.mfcc.hopsize = 256;   
p.mfcc.num_ceps_coeffs = 20;
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

if 0, %% compute features
    disp('-- starting feature extraction')
    t0 = cputime;
    i0 = 1;
    for i=i0:length(data.class),
        t1 = cputime;
        if i-i0, %% estimate remaining and total time
            disp([num2str(i),'/',num2str(length(data.class)), ...
                    ' FE estrem [m]: ',num2str((t1-t0)/(i-i0)*(length(data.class)-i)/60), ...
                    ' FE esttot [m]: ',num2str((t1-t0)/(i-i0)*length(data.class)/60)])
        end
        [wav, fs] = audioread(data.list_wavs{i});
        siz = size(wav);
        if fs ~= p.fs,
            error(['sampling frequency is ',num2str(fs),' and not ',num2str(p.fs), ' for file: ', data.list_wavs{i}]);
        end
        if siz(1)>p.fs*120,
            x0 = ceil(siz(1)/2-p.fs*60);
            x1 = floor(siz(1)/2+p.fs*60);
        else
            x0 = 1;
            x1 = siz(1);
        end
        if siz(1)<p.fs*30,
            disp(['WARNING: file is shorter than 30 seconds: ',data.list_wavs{i}]);
        end
        
        if 1,
            disp('wavread')
            wav = audioread(data.list_wavs{i},[x0 x1]);

            disp('ma_sone')
            sone = ma_sone(wav,p.sone);
            disp('ma_sh')
            sh{i} = ma_sh(sone,p.sh.hist_res);
            disp('ma_fp')
            fp{i} = ma_fp(sone,p.fp);
            disp('ma_ph')
            ph{i} = ma_ph(sone,p.ph);

            disp('ma_mfcc')
            mfcc = ma_mfcc(wav,p.mfcc);
            disp('ls')
            ls{i} = ma_fc(mfcc,p.ls);
            disp('ap')
            ap{i} = ma_fc(mfcc,p.ap);
        end
    end
end

if 1, %% compute distance matrices
    disp('-- starting distance computations')
    D_sh = zeros(length(data.class));
    D_fp = zeros(length(data.class));
    D_ph = zeros(length(data.class));
    D_ap = zeros(length(data.class));
    D_ls = zeros(length(data.class));

    t0 = cputime;
    n = length(data.class)*(length(data.class)-1)/2;
    k = 0;
    for i=1:length(data.class)-1,
        for j=i+1:length(data.class),
            t1 = cputime;
            if k,
                disp([num2str(k),'/',num2str(n), ...
                        ' DC estrem [m]: ',num2str((t1-t0)/k*(n-k)/60), ...
                        ' DC esttot [m]: ',num2str((t1-t0)/k*n/60)])
            end
            k = k+1;
            D_sh(i,j) = norm(sh{i}(:)-sh{j}(:));
            D_fp(i,j) = norm(fp{i}(:)-fp{j}(:));
            D_ph(i,j) = norm(ph{i}(:)-ph{j}(:));
            D_ap(i,j) = ma_cms(ap{i},ap{j},p.ap);
            D_ls(i,j) = ma_cms(ls{i},ls{j},p.ls);
        end
    end

    %% symmetric
    D_sh = D_sh + D_sh';
    D_fp = D_fp + D_fp';
    D_ph = D_ph + D_ph';
    D_ls = D_ls + D_ls';
    D_ap = D_ap + D_ap';    
end

%% R-precision
%%   precision (#relevant/R) at R where R is the number 
%%   of relevant songs for the query
k = 1;
rp_sh = zeros(length(data.class),1);
rp_ap = zeros(length(data.class),1);
rp_ls = zeros(length(data.class),1);
rp_fp = zeros(length(data.class),1);
rp_ph = zeros(length(data.class),1);
rp_rnd = zeros(length(data.class),1);
for i=1:length(dirs),
    idx = find(data.class == i);
    R = length(idx)-1;
    for j=idx,
        [dummy rank] = sort(D_sh(j,:));
        rp_sh(k) = (length(intersect(rank(1:R+1),idx))-1)/R;
        [dummy rank] = sort(D_ap(j,:));
        rp_ap(k) = (length(intersect(rank(1:R+1),idx))-1)/R;
        [dummy rank] = sort(D_ls(j,:));
        rp_ls(k) = (length(intersect(rank(1:R+1),idx))-1)/R;
        [dummy rank] = sort(D_ph(j,:));
        rp_ph(k) = (length(intersect(rank(1:R+1),idx))-1)/R;
        [dummy rank] = sort(D_fp(j,:));
        rp_fp(k) = (length(intersect(rank(1:R+1),idx))-1)/R;
        rank = [j,randperm(length(data.class))];
        rp_rnd(k) = (length(intersect(rank(1:R+1),idx))-1)/R;        
        k = k+1;
    end
end

mean(rp_ap)
mean(rp_ls)
mean(rp_fp)
mean(rp_sh)
mean(rp_ph)

figure; 
subplot(4,1,1); plot(rp_sh) 
subplot(4,1,2); plot(rp_fp)
subplot(4,1,3); plot(rp_ph)
subplot(4,1,4); plot(rp_ls); hold on; plot(rp_ap,'r')
