

% rot directory in which subdirectories are located

%rootdir = 'C:\Users\jeff\Google Drive\ECEN5322\Final_project_data\Music Data\tracks\';
rootdir = 'C:\Users\Johnny_Five\Google Drive\ECEN5322\Final_project_data\Music Data\tracks\';

% sample rate of all wav files in the directories
fs = 11025; 

%data path for pre extracted data
data_path = 'C:\Users\Johnny_Five\Google Drive\ECEN5322\Final_project_data\';

data = data_path_extraction(rootdir);

%strcar local data path with the data to be used
x = strcat(data_path, 'Transformed_data\mfcc20_full.mat');

%only load mfcc if it isnt alread loaded
if(exist('mfcc'))
   disp('mfcc already loaded');
else
    load(x);
end

% data paths and paramater setup
data = data_path_extraction(rootdir);       
p = create_ma_parameters(fs);




%mfcc full song
if 0
    for i=1:length(data.list_wavs)
        [wav, fs] = audioread(data.list_wavs{i});
        mfcc{i} =  ma_mfcc(wav, p.mfcc);
        disp(i)
    end
end

%30 gaussian clustering
if 0
    for i=1:length(data.list_wavs)
        
        %similarity using gmm with 30 clusters
        cm_gmm(i) = ma_fc(mfcc{i}, p.ap);
        
        fprintf('cm_gmm %g \n',i);
        
    end
end

%5 gaussian clustering
if 1
    p.ap.num_clusters = 5;
     %get 5 gaussian clusters
     for i=1:length(data.list_wavs)
        cm_gmm(i) = ma_fc(mfcc{i}, p.ap);
        fprintf('gmm5 %d\n', i);
     end
     
     %vectorize centers and covars
     gmm5_vectors = zeros(length(data.list_wavs), 200);
     for i=1:length(data.list_wavs)
         centers = zeros(1,100);
         covars = zeros(1,100);
         for j=1:5
             centers(1,((j-1)*20+1):(j*20)) = cm_gmm(i).centres(j,:); 
             covars(1,((j-1)*20+1):(j*20)) = cm_gmm(i).covars(j,:);
         end
         %place into final vector
         gmm5_vectors(i,1:100) = centers;
         gmm5_vectors(i,101:200) = covars;
          fprintf('vectorize %d\n', i);
     end
     %reset to defaults
     p.ap.num_clusters = 30;
end


%zero crossing 
if 0
    clear Z;
    H = dsp.ZeroCrossingDetector;
    for i=1:length(data.list_wavs)
        [wav, fs] = audioread(data.list_wavs{i});
        Z(i,2) = length(wav)/fs;
        Z(i,1) = step(H, wav); 
        Z(i,3) = Z(i,2)/Z(i,1)*1000;
        release(H);
        disp(i)
    end
    zero_cross = Z(:,3);
end

%fluctuation patterns
if 0
    for i = 1:length(data.list_wavs)
        fp_full{i} =ma_fp(squeeze(mfcc{i}), p.fp);
        disp(i);
    end
end

%distance matrix hacked in for now
if 0
    for i = 1:length(data.list_wavs)
        for j = 1:length(data.list_wavs)
            fp_dist(i,j) = norm(fp_full{i}- fp_full{j});
        end
        disp(i);
    end    
end



%power spectrum
%not finished
if 0
    seg_size = 512;
    hop_size = 256;
    
    
    w = hann(seg_size);
    
    for i=1:1%length(data.list_wavs)
        [wav, fs] = audioread(data.list_wavs{i});
        num_segments = floor((length(wav)-seg_size)/hop_size)+1;
        P = zeros(seg_size/2+1, num_segments); %%malloc
        for i=1:num_segments
            idx = (1:seg_size)+(i-1)*hop_size;
            x = abs(fft(wav(idx).*w)/sum(w)*2).^2;
            P(:,i) = x(1:end/2+1);
        end    
    end
end




    























