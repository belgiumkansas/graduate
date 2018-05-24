function [ph, ph_low, p] = ma_ph(sone, p)
%%
%% compute Periodicity Histogram (PH)
%%   see Pampalk et al. ISMIR'03 for details
%%
%% [ph, ph_low, p] = ma_ph(sone,p)
%%
%% INPUT
%%   sone (matrix) as returned from "ma_sone" (size: critical-bands x time)
%%   parameter structure p
%%       p.hist_res = 30; %% energy resolution in histogram (used 50 for ismir03)
%%       p.sequence.length = 1024; %% ~12 sec @ 11025, size of sequence (window) to 
%%                                 %% analyze periodicities
%%       p.sequence.hopsize = 512; %% smaller values make sense, 
%%       p.sequence.windowfunction = 'hann';
%%       p.fft_hopsize = 256; %% (~12ms @ 11kHz) hopsize used to create 
%%                            %% sonogram (see ma_sone)
%%       p.acc = 5;       %% ismir version orig: 5 (recommended for 12ms hopsize in 
%%                        %% sone), combfilter accuracy
%%       p.minbpm = 40;   %% ismir version orig: 40, minimum bpm to look for
%%       p.maxbpm = 240;  %% ismir version orig: 240
%%       p.hist_res = 30; %% resolution of histogram
%%       p.maxval = 0.4;  %% highest energy to expect in histogram
%%       p.minval = 0;    %% 1/p.hist_res
%%       p.visu = 0;      %% do some visualizations
%%       p.preferred_tempo = 'moleants'; %% {'moleants' | 'none'}
%% OUTPUT
%%   ph (matrix) size: critical-bands x hist_res
%%   ph_low (matrix), same as ph but only for the lowest frequency bands
%%
%% note: to compute distances between two PHs use Euclidean distance, e.g.
%%       distance = norm(ph1(:)-ph2(:));
%%       however, PHs best serve as starting point for higher level descriptors e.g. 
%%       using sum(ph(:)) (= total energy contained in periodic beats)

%% elias 1.6.2004

if ~nargin,
    disp('testing: ma_ph')    
        
    p.type = 'ph';
    p.fs = 11025;

    %% sonogram
    p.hopsize = 256;  
    p.fft_size = 512; 
    p.outerear = 'modified_terhardt'; %% {'terhardt' | 'modified_terhardt' | 'none'}
    p.visu = 0;    

    freq = repmat([150,0,150,0,350,0,150,0,0,0,350,0],1,12);
    wav = ma_test_create_wav(p.fs,freq,.2,0.2);
    
    sone = ma_sone(wav,p);

    p.sequence.length = 1024;
    p.sequence.hopsize = 512; %% smaller values make sense
    p.sequence.windowfunction = 'hann';
    p.fft_hopsize = p.hopsize; %% (~12ms @ 11kHz) hopsize used to create sone
    p.acc = 5;      %% ismir version orig: 5 (recommended for 12ms hopsize in sone)
    p.minbpm = 40;  %% ismir version orig: 40
    p.maxbpm = 240; %% ismir version orig: 240
    p.hist_res = 30;
    p.maxval = 0.4;
    p.minval = 0; %% 1/p.hist_res        
    p.visu = 1;
    p.preferred_tempo = 'moleants'; %% {'moleants' | 'none'}

    [ph, ph_low] = ma_ph(sone,p);
        
    figure; 
    subplot(3,1,1); set(gca,'fontsize',8);
    imagesc(sone); set(gca,'ydir','normal','xtick',[])
    title('Sonogram')
    subplot(3,1,2); set(gca,'fontsize',8);
    imagesc(p.minbpm+p.acc:p.acc:p.maxbpm,1:size(sone,1),ph); 
    set(gca,'ydir','normal','xtick',[])
    title('PH')
    subplot(3,1,3); set(gca,'fontsize',8);
    imagesc(p.minbpm+p.acc:p.acc:p.maxbpm,1:size(sone,1),ph_low); set(gca,'ydir','normal')
    title('PH low')
    
    sound(wav,p.fs);
    ph = 'done'; %% dont flood command window with numbers
    return    
end

if size(sone,2)<p.sequence.length, %% pad with zeros (need this for my 10sec collection)
    tmp.missing = p.sequence.length-size(sone,2);
    sone = [zeros(size(sone,1),ceil(tmp.missing/2)), ...
            sone,zeros(size(sone,1),ceil(tmp.missing/2))];
end

cb = size(sone,1); %% number of critical-bands

sone = sone./max(sone(:)); %% normalize
sone = gradient(sone,1,2); %% difference filter to emphasize attacks
sone(sone<0) = 0;          %% half-wave rectified (only interested in attacks and not decays)

%% mem alloc
e = zeros(cb,length(p.minbpm:p.acc:p.maxbpm));          %% comb filter output
ph = zeros(p.hist_res,length(p.minbpm:p.acc:p.maxbpm)); %% histogram

f = (p.minbpm:p.acc:p.maxbpm)/60; %% bpm to Hz
if strcmp(p.preferred_tempo,'moleants'),
    f0 = 2.08; beta = 4.0; %% moleants, prefered tempo, resonance model
    x2 = 1./(sqrt((f0.^2 - f.^2).^2 + beta*f.^2)) - 1./(sqrt(f0.^4 + f.^4));
    x2 = x2./max(x2); 
else %% no weighting (x2 = 1)
    x2 = f*0+1;
end

switch p.sequence.windowfunction,
    case 'hann', w = hann(p.sequence.length);
    case 'hamming', w = hamming(p.sequence.length);
    case 'boxcar', w = boxcar(p.sequence.length);
    otherwise, error(['unkown window function: ',p.sequence.windowfunction]);
end

c.dftfils = get_filts(p,w); %% get combfilters

idx = 1:p.sequence.length;

seq_k = 0;
while idx(end) <= size(sone,2),
    seq_k = seq_k+1; %% count sequence
    tmp.sone = sone(:,idx).*repmat(w',cb,1);
    dft = fft(tmp.sone,p.sequence.length,2)'; %% get signal in frequency domain
    
    k = 0;
    for bpm = p.minbpm:p.acc:p.maxbpm, 
        k = k+1;        
        for j = 1:cb
            x = (abs(c.dftfils(:,k).*dft(:,j))); %% apply comb filter
            e(j,k) = sum(x(2:end));
        end
    end
    
    if p.visu, %% PH results for each sequence
        figure 
        sp(1) = subplot(5,1,1); 
        set(gca,'fontsize',8)
        imagesc(tmp.sone); set(gca,'xtick',[]);
        title('hann weighted (half-wave rectified(diff(sequence)))')
        ylabel('Bark')
        
        sp(2) = subplot(5,1,2); %% comb filter response
        set(gca,'fontsize',8)
        imagesc(p.minbpm:p.acc:p.maxbpm,1:cb,e); h=colorbar; 
        set(gca,'xtick',[])
        set(h,'fontsize',8)
        title('combfilter response')
    end
    
    e = e.*repmat(x2,size(sone,1),1); 
    
    if p.visu,
        sp(3) = subplot(5,1,3); %% moleants weighting
        set(gca,'fontsize',8)
        imagesc(p.minbpm:p.acc:p.maxbpm,1:cb,e); h=colorbar;
        set(h,'fontsize',8)
        set(gca,'xtick',[])
        title('preferred tempo weighting')
    end
    
    e = abs(diff(e,1,2));              
    
    if p.visu,
        sp(4) = subplot(5,1,4); %% fullwave rectified diff
        set(gca,'fontsize',8)
        imagesc(p.minbpm:p.acc:p.maxbpm,1:cb,e); h=colorbar; 
        set(h,'fontsize',8)
        set(gca,'xtick',[])
        title('full-wave rectified(diff(combfilter response))')
    end
    
    e_low = sum(e(1:3,:),1); %% sum over all low-freq bands
    e = sum(e,1); %% sum over all bands
    
    if p.visu,
        sp(5) = subplot(5,1,5); 
        set(gca,'fontsize',8)
        plot(p.minbpm+p.acc:p.acc:p.maxbpm,e); set(gca,'xlim',[p.minbpm+p.acc p.maxbpm]);
        h = colorbar; set(h,'visible','off')
        set(sp,'ydir','normal')
        for i=1:length(sp),
            pos = get(sp(i),'position');
            set(sp(i),'position',pos.*[1 1 1 0.7])
        end
        title('sum over all critical-bands')
        ylabel('bpm')
    end
    
    centers = linspace(p.minval,p.maxval,p.hist_res);
    e_gram = zeros(p.hist_res,length(p.minbpm+p.acc:p.acc:p.maxbpm));

    ph_all{seq_k} = e;
    ph_all_low{seq_k} = e_low; %% to describe periodicities in low frequencies
    
    idx = idx + p.sequence.hopsize;    
end    

e = vertcat(ph_all{:});
e_low = vertcat(ph_all_low{:});

for i = 1:p.hist_res,
    for j = 1:length(p.minbpm+p.acc:p.acc:p.maxbpm),
        ph(i,j) = sum(e(:,j)>=centers(i));
        ph_low(i,j) = sum(e_low(:,j)>=centers(i)/6);
    end
end

%% normalize wrt length of spectrogam
ph = ph./size(e,1);         
ph_low = ph_low./size(e,1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function dftfils = get_filts(p,w)
%% create filters for combfilter

dftfils = zeros(p.sequence.length,length(p.minbpm:p.acc:p.maxbpm));

k=0;
for bpm = p.minbpm:p.acc:p.maxbpm,
    k=k+1;
    fil = zeros(p.sequence.length,1);
    
    %% set every hopsize samples of filter to 1 and 0 else
    npulses = ceil((bpm/60 * p.sequence.length/p.fs*p.fft_hopsize));
    hopsize = floor(p.sequence.length / npulses);
    fil(1 + ((0:npulses-1)*hopsize)) = 1/npulses;
    
    %% get filter in the frequency domain
    dftfils(:,k) = fft(fil.*w./sum(w)/2);
end
