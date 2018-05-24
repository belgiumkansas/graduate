function fp = ma_fp(sone,p)
%%
%% compute Fluctuation Pattern (FP)
%%   see Pampalk et al. ACM-MM'02 for details
%%   december 2001 version (islands of music, master's thesis)
%%
%% fp = ma_fp(sone,p)
%%
%%
%% INPUT
%%   sone (matrix) as returned from "ma_sone" (size: critical-bands x time)
%%   parameter structure p
%%       p.sequence.length = 512;   %% windows size (ca 6sec @ 11kHz with 
%%                                  %% 128 sone hopsize)
%%       p.sequence.hopsize = 256;  
%%       p.sequence.windowfunction = 'boxcar';
%%       p.fs = 11025;              %% sampling frequency of wav file
%%       p.fft_hopsize = 128;       %% (~12ms @ 11kHz) hopsize used to create sone 
%%       p.visu = 0;                %% do some visualizations
%% OUTPUT
%%   fp (matrix) size: critical-bands x 60 (modulation frequencies)
%%
%% note: to compute distances between two FPs use Euclidean distance, e.g.
%%       distance = norm(fp1(:)-fp2(:));

%% elias 1.6.2004

if ~nargin,
    disp('testing: ma_fp')    
    
    p.fs = 11025;
    
    p.sone.type     = 'spectrogram';
    p.sone.visu     = 0;
    p.sone.sone     = 1;
    p.sone.spread   = 1;
    p.sone.outerear = 'modified_terhardt'; %% {'terhardt' | 'modified_terhardt' | 'none'}
    p.sone.fft_size = 256;  %% ca 23ms @ 11kHz
    p.sone.hopsize  = 128;  %% ca 12ms @ 11kHz
    p.sone.fs       = p.fs;
        
    p.fp.sequence.length = 512;
    p.fp.sequence.hopsize = 256;
    p.fp.sequence.windowfunction = 'boxcar';
    p.fp.fs = p.fs;                    %% sampling frequency of wav file
    p.fp.fft_hopsize = p.sone.hopsize; %% (~12ms @ 11kHz) hopsize (aka overlap) used to create sone 
    p.fp.visu = 1;
        
    freq = repmat([150,550,150,0,150,0],1,6);
    wav = ma_test_create_wav(p.fs,freq,.2,0.2);

    sone = ma_sone(wav,p.sone);    
    fp = ma_fp(sone,p.fp);

    figure; 
    subplot(2,1,1); 
    set(gca,'fontsize',8);
    imagesc(sone); set(gca,'ydir','normal')
    title('Sonogram')
    subplot(2,1,2); 
    set(gca,'fontsize',8);
    imagesc(linspace(0,10,size(fp,2)),1:size(fp,1),fp); set(gca,'ydir','normal')
    title('Fluctuation Pattern')
    xlabel('Modulation Frequency [Hz]')
    ylabel('Bark')

    %sound(wav,p.sone.fs);
    fp = 'done'; %% dont flood command window with numbers
    return    
end

%% constants
c.f = p.fs/p.fft_hopsize*(0:p.sequence.length/2)/p.sequence.length;
[tmp.dummy, c.f10] = min(abs(c.f-10));                                  %% find freq (index) closest to 10Hz
c.f10 = c.f10 + 2;                                                      %% one extra which is lost through diff, one extra because first coeff is ignored
c.flux_w = repmat(1./(c.f(2:c.f10)/4+4./c.f(2:c.f10)),size(sone,1),1);  %% see zwicker book p. 254
c.gauss_w = [.05 .1 0.25 0.5 1 0.5 .25 0.1 0.05];                       %% smoothing filter, original as used in IoM master's thesis
c.gauss_w2 = interp1(1:length(c.gauss_w), c.gauss_w, ...
    linspace(1,length(c.gauss_w),ceil(length(c.gauss_w)*c.f10/62)), 'PCHIP');
c.blur = filter2(c.gauss_w,eye(size(sone,1)));
c.blur = c.blur./repmat(sum(c.blur,2),1,size(sone,1));
c.blur2 = filter2(c.gauss_w2,eye(c.f10-2));
c.blur2 = c.blur2./repmat(sum(c.blur2,2),1,c.f10-2);

if p.visu,
    figure; %% flux model and blur filters
    subplot(1,3,1); 
    set(gca,'fontsize',8)
    plot(c.f(2:c.f10),c.flux_w(1,:)); axis tight
    title('Fluctuation Strength')
    xlabel('Hz'); ylabel('Weighting')
    
    subplot(1,3,2); 
    set(gca,'fontsize',8)
    imagesc(c.blur); set(gca, 'ydir', 'normal'); colormap gray
    title('Smoothing Filter')
    xlabel('Bark')
    
    subplot(1,3,3); 
    set(gca,'fontsize',8)
    imagesc(c.blur2); set(gca, 'ydir', 'normal'); colormap gray    
    title('Smoothing Filter')
    xlabel('Modulation Frequency')
end

if size(sone,2)<p.sequence.length, %% pad with zeros (need this for my 10sec collection)
    tmp.missing = p.sequence.length-size(sone,2);
    sone = [zeros(size(sone,1),ceil(tmp.missing/2)), ...
            sone,zeros(size(sone,1),ceil(tmp.missing/2))];
end

idx = 1:p.sequence.length;

switch p.sequence.windowfunction,
    case 'hann', w = hann(p.sequence.length);
    case 'hamming', w = hamming(p.sequence.length);
    case 'boxcar', w = boxcar(p.sequence.length);
    otherwise, error(['unkown window function: ',p.sequence.windowfunction]);
end

k = 0;
while idx(end) <= size(sone,2),
    k = k+1; %% count sequence
    tmp.wsone = sone(:,idx).*repmat(w',size(sone,1),1);

    X = fft(tmp.wsone,p.sequence.length,2);
    X = abs(X(:,2:c.f10)); %% amplitude spectrum (not power)
    
    if p.visu,
        figure; %% results of flux calculation for each sequence

        sp(1) = subplot(5,1,1); %% original sequence        
        set(gca,'fontsize',8)
        imagesc(tmp.wsone); set(gca, 'ydir', 'normal','xtick',[]); 
        title('sonogram (bark vs time vs sone)')
        sp(2) = subplot(5,1,2); %% fft
        set(gca,'fontsize',8)
        imagesc(X); set(gca, 'ydir', 'normal','xtick',[]); 
        title('Modulation Frequencies (FFT)')
    end
    
    X = X.*c.flux_w;

    if p.visu,
        sp(3) = subplot(5,1,3); %% flux weighting
        set(gca,'fontsize',8)
        imagesc(X); set(gca, 'ydir', 'normal','xtick',[]); 
        title('Weighted Fluctuation Strength')
    end

    X = abs(diff(X,1,2));

    if p.visu,
        sp(4) = subplot(5,1,4); %% full wave rectified
        set(gca,'fontsize',8)
        imagesc(X); set(gca, 'ydir', 'normal','xtick',[]); 
        title('Full-Wave Rectified Difference Filter')
    end
    
    X = c.blur*X*c.blur2';

    if p.visu,
        sp(5) = subplot(5,1,5); %% blured
        imagesc(linspace(0,10,size(X,2)),1:size(X,1),X); set(gca, 'ydir', 'normal'); 
        set(sp,'fontsize',8)
        title('Smoothing Filter')
        xlabel('Modulation Frequency [Hz]')
        for i=sp,
            set(i,'position',get(i,'position').*[1 1 1 0.8]);
        end
    end
    
    fp_all{k} = reshape(interp1(X',linspace(1,size(X,2),60))',1,60*size(sone,1)); %% aka mfs60
    
    idx = idx + p.sequence.hopsize;    
end    
    
fp =  reshape(median(vertcat(fp_all{:}),1),size(sone,1),60);