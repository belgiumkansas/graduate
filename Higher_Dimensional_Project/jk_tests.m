%% These are experiments to analyse different features for the tracks
 clear all
 fs = 11025; %% sample rate of all wav files in the directories
 addpath('ma');
 addpath('netlab');
    %% root directory in which subdirectories are located
    
    %data path for pre extracted data
    data_path = 'C:\Users\hj\Google Drive\Final_project_data\Transformed_data\';
    %rootdir = 'C:\Users\jeff\Google Drive\ECEN5322\Final_project_data\Music Data\tracks\';
    %rootdir = 'C:\Documents and Settings\Elias\Desktop\ma\ma\ma\test\';
    %rootdir = 'C:\Users\Johnny_Five\Google Drive\ECEN5322\Final_project_data\Music Data\tracks\';
    rootdir = 'C:\Users\hj\Google Drive\Final_project_data\Music Data\tracks\';
 
    data = data_path_extraction(rootdir);

    p = create_ma_parameters(fs);

% calcualting zero crossing rate per ms
if 0
    for i=1:length(data.list_wavs)
        [wav, fs] = audioread(data.list_wavs{i});
        %ZCR(i) = (mean(abs(diff(sign(wav))))/fs)*1e3;
        zcr(i) = (sum(abs(diff(sign(wav))))/2/length(wav)*fs)*1e-3;
        
    end
end



% some shitty xperiments
if 0
    M_dB = squeeze(mfcc_mid_19);

    t = zeros(1,36); 
    t(1) = 1; t( 7: 8) = 5; t(15:18) = 9;
    t(2) = 2; t( 9:10) = 6; t(19:20) = 10;
    t(3:4) = 3; t(11:12) = 7; t(24:29) = 11;
    t(5:6) = 4; t(13:14) = 8; t(30:36) = 12;

    mel2 = zeros(12,817);%size(M_dB,2));
    for i=1:12
        mel2(i,:) = sum(M_dB(t==i,:),1);
    end
    
    f = linspace(0,22050/512/2,64+1); 
    flux = repmat(1./(f(2:32)/4+4./f(2:32)),12,1);
    w = [0.05, 0.1, 0.25, 0.5, 1, 0.5, 0.25, 0.1, 0.05];
    filt1 = filter2(w,eye(12)); %% 12x12
    filt1 = filt1./repmat(sum(filt1,2),1,12);
    filt2 = filter2(w,eye(30)); %% 30x30
    filt2 = (filt2./repmat(sum(filt2,2),1,30));
end

% 30 Gaussian clustering and similarity computation (Monte carlo sampling)
if 1
    load(strcat(data_path, 'mfcc19_mid_19.mat'));
    for i =1 :729
        cm(i) = ma_fc(squeeze(mfcc_mid_19(i,:,:)),p.ap);
    end
%     for i = 1:729
%         for j = 1:729
%             distance(i,j) = ma_cms(cm(i),cm(j),p.ap);
%         end
%     end
end

% calculating sonogram for mid 19 seconds
if 0
    
    for i=1:3%length(data.list_wavs)

    [wav, fs] = audioread(data.list_wavs{i});
    size = length(wav);    
    if(size < fs*19+1)
        sample = wav;
        start = 1;
        ending = size-1;
    else
        start = floor(size/2 - fs*9.5);
        ending = floor(size/2 + fs*9.5);
    end

     sone_mid_19(i,:,:) = ma_sone(wav(start:ending), p.sone);
     
    end

    
end

% calculating fluctuation pattern for mid 19 seconds
if 0
    
    for i =1 :3%729
        fp_mid_19(i,:,:) = reshape(ma_fp(squeeze(sone_mid_19(i,:,:)), p.fp),[1,20,60]);
        %fp_mid_19(i,:,:) = reshape(ma_fp(reshape(sone_mid_19(i,:,:),[20,817]), p.fp),[1,20,60]);
    end
end
  
% calculation of rms ( indication of loudness)

if 0
    
    for i=1:length(data.list_wavs)
        [wav, fs] = audioread(data.list_wavs{i});
        rms(i) = sqrt(mean(wav.^2));             
    end
    
end

% average loudness

if 0
    
    for i=1:length(data.list_wavs)
        
        avg_loudness(i) = mean(mfcc_mid_19(i,:));             
    end
    
end

% Percussivness
if 0
    
    for i=1:length(data.list_wavs)      
          
        percussiveness(i) = mean(abs(diff(mfcc_mid_19(i,:),1,2)));
        
    end
    
end

% yet to complete
% bass,aggressiveness, Domination of Low Frequency

if 0
    
    for i =1 :729
        bass(i) = fp_mid_19(i,:,:) 
    end
end








    


