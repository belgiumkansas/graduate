function sh = ma_sh(sone,hist_res)
%%
%% compute Spectrum Histogram (SH)
%%   very simple approach to summarize the sonogram
%%   see Pampalk et al. ISMIR'03 for details
%%
%% sh = ma_sh(sone,hist_res)
%% 
%% USAGE
%%   [wav, p.fs] = wavread(myfile);
%%   sone = ma_sone(wav,p);
%%   sh = ma_sh(sone,hist_res)
%%
%%
%% INPUT
%%   sone (matrix) as returned from "ma_sone" (size: critical-bands x time)
%%   hist_res = 25; %% loudness resolution in histogram (used 50 for ismir03)
%%
%% OUTPUT
%%   sh (matrix) size: critical-bands x hist_res
%%
%% note: to compute distances between two SHs use Euclidean distance, e.g.
%%       distance = norm(sh1(:)-sh2(:));

%% elias 31.5.2004

if ~nargin,
    disp('testing: ma_sh')        
    
    p.fs = 22050;
    p.hist_res = 25;
    wav = ma_test_create_wav(p.fs);
    sone = ma_sone(wav,p);

    sh = ma_sh(sone, p.hist_res);

    figure;     
    subplot(2,1,1); set(gca,'fontsize',8);
    imagesc(sone); set(gca,'ydir','normal','xtick',[])
    title('Sonogram'); xlabel('Time'); ylabel('Bark')

    subplot(2,1,2); set(gca,'fontsize',8);
    imagesc(sh);    set(gca,'ydir','normal','xtick',[])
    title('Spectrum Histogram'); xlabel('Loudness Level'); ylabel('Bark')    
    
    colormap hot

    %sound(wav,p.fs);
    
    sh = 'done'; %% dont flood command window with numbers
    return;
end

sone = sone./max(sone(:)); %% normalize

cb = size(sone,1);
sh = zeros(cb,hist_res);

centers = linspace(1/hist_res,1,hist_res);
for i = 1:hist_res,
    for j = 1:cb,
        sh(j,i) = sum(sone(j,:)>=centers(i));
    end
end    

sh = imfilter(sh,fspecial('gaussian'),'replicate');

sh = sh./sum(sh(:)); %% normalize histogram
