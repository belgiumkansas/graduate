function [mfcc,DCT] = ma_mfcc(wav, p)
%%
%% Compute Mel Frequency Cepstrum Coefficients.
%%   [mfcc,DCT] = ma_mfcc(wav, p)
%%
%% USAGE
%%   ma_mfcc;                         %% test mode
%%   mfcc = ma_mfcc(wav, p);          %% normal usage
%%   [mfcc, DCT] = ma_mfcc(wav, p);   %% DCT matrix is needed to decode the 
%%                                    %% coefficients (e.g. for visualization)
%%
%% based on function "mfcc" in Auditory Toolbox by Malcolm Slaney 
%% (http://www.slaney.org/malcom) see Auditory Toolbox for details
%%
%% INPUT
%%   wav (vector) obtained from wavread or ma_mp3read 
%%                (use mono input! 11kHz recommended)
%%   p (struct) parameters e.g. 
%%      p.fs                = 11025; %% sampling frequency of given wav (unit: Hz)
%%      * p.visu            = 0;     %% create some figures
%%      * p.fft_size        = 256;   %% (unit: samples) 256 are about 23ms @ 11kHz 
%%      * p.hopsize         = 128;   %% (unit: samples) aka overlap
%%      * p.num_ceps_coeffs = 20;   
%%      * p.use_first_coeff = 1;     %% aka 0th coefficient (contains information 
%%                                   %% on average loudness)
%%      * p.mel_filt_bank   = 'auditory-toolbox'; 
%%                                   %% mel filter bank choice 
%%                                   %% {'auditory-toolbox' | [f_min f_max num_bands]}
%%                                   %% e.g. [20 16000 40], (default)
%%                                   %% note: auditory-toobox is optimized for 
%%                                   %%       speech (133Hz...6.9kHz)
%%      * p.dB_max          = 96;    %% max dB of input wav (for 16 bit input 96dB is SPL)
%%    all fields marked with * are optional (defaults are defined)
%% 
%% OUTPUT
%%   mfcc (matrix) size coefficients x frames
%%   DCT (matrix) 

%% elias 13.6.2004

if nargin == 0, %% test mode
    disp('testing: ma_mfcc')        
    
    p.type            = 'mfcc';
    p.visu            = 1;
    p.fft_size        = 512*2;
    p.hopsize         = 256;
    p.num_ceps_coeffs = 20;
    p.use_first_coeff = 1;
    p.fs              = 22050*2;
    p.mel_filt_bank   = [20 20000 40]; %% {'auditory-toolbox' | [f_min f_max num_bands]}
                                            %% e.g. [20 16000 40]
    wav = ma_test_create_wav(p.fs);
    
    %sound(wav,p.fs);
    
    mfcc = ma_mfcc(wav, p);
        
    mfcc = 'done'; %% dont flood command window with mfcc numbers
    return    
end

%% defaults
if ~isfield(p,'type'),      p.type = 'sone';         end
if ~isfield(p,'fs'),        error('sampling frequency (p.fs) not specified'); end
if ~isfield(p,'mel_filt_bank'),  p.mel_filt_bank = [20 16000 40]; end
if ~isfield(p,'dB_max'),    p.dB_max = 96;               end
if ~isfield(p,'do_visu'),   p.do_visu = 0;               end
if ~isfield(p,'fft_size'),  p.fft_size = p.fs/11025*256; end
if ~isfield(p,'overlap'),   p.overlap = p.fs/11025*128;  end
if ~isfield(p,'num_ceps_coeffs'), p.num_ceps_coeffs = 40; end
if ~isfield(p,'use_first_coeff'), p.use_first_coeff = 40; end
c.fft_freq = linspace(0,p.fs/2,p.fft_size/2+1);

if strcmp(p.mel_filt_bank,'auditory-toolbox'),
    %% frequency scale parameters (filterbank, auditory toolbox, Malcolm Slaney)
    %% comment: center frequency of first critical band is located at about 200Hz!!
    c.low_freq = 400/3;
    c.num_lin_filt = 13;
    c.num_log_filt = 27;
    c.num_filt = c.num_lin_filt + c.num_log_filt;
    c.lin_spacing = 200/3;
    c.log_spacing = 1.0711703;
    
    c.freqs = c.low_freq + (0:c.num_lin_filt-1)*c.lin_spacing;
    c.freqs(c.num_lin_filt+1:c.num_filt+2) = ...
        c.freqs(c.num_lin_filt) * c.log_spacing.^(1:c.num_log_filt+2);
else 
    c.num_filt = p.mel_filt_bank(3);
    tmp.f = p.mel_filt_bank(1):p.mel_filt_bank(2); %% use same spectrum as for bark (according to zwicker & fastl)
    tmp.mel = log(1+tmp.f/700)*1127.01048;
    
    tmp.m_idx = linspace(1,max(tmp.mel),c.num_filt+2); 
        
    tmp.f_idx = (1:c.num_filt+2)*0;
    for i=1:c.num_filt+2,
        [tmp.dummy tmp.f_idx(i)] = min(abs(tmp.mel - tmp.m_idx(i)));
    end
    c.freqs = tmp.f(tmp.f_idx);
end    

c.lower  = c.freqs(1:c.num_filt);
c.center = c.freqs(2:c.num_filt+1);
c.upper  = c.freqs(3:c.num_filt+2);

%% ignore filters outside of spectrum
[tmp.dummy, tmp.idx] = max(find(c.center <= p.fs/2));
c.num_filt = min(tmp.idx,c.num_filt);

mfccFilterWeights = zeros(c.num_filt,p.fft_size/2+1);
c.triangleHeight = 2./(c.upper-c.lower);

for i=1:c.num_filt,
    mfccFilterWeights(i,:) = ...
        (c.fft_freq > c.lower(i) & c.fft_freq <= c.center(i)).* ...
        c.triangleHeight(i).*(c.fft_freq-c.lower(i))/(c.center(i)-c.lower(i)) + ...
        (c.fft_freq > c.center(i) & c.fft_freq < c.upper(i)).* ...
        c.triangleHeight(i).*(c.upper(i)-c.fft_freq)/(c.upper(i)-c.center(i));
end

DCT = 1/sqrt(c.num_filt/2)*cos((1-p.use_first_coeff:(p.num_ceps_coeffs-1))' * (2*(0:(c.num_filt-1))+1) * pi/2/c.num_filt);
if p.use_first_coeff, 
    DCT(1,:) = DCT(1,:) * sqrt(2)/2;
end

if p.visu,
    figure; %% mel scale and DCT
    
    subplot(2,1,1); %% mel filter bank
    set(gca,'fontsize',8)
    semilogx(c.fft_freq,mfccFilterWeights'); %axis([0 c.upper(c.num_filt) 0 max(mfccFilterWeights(:))])
    title('Filterbank');
    
    subplot(2,1,2); %% DCT
    imagesc(DCT'); set(gca,'ydir','normal','fontsize',8); xlabel('MFC-Coeff'); ylabel('Mel Band'); colormap gray
    title('DCT');
end

%% figure out number of frames
frames = 0;
idx = p.fft_size;
while idx<=length(wav),
    frames = frames + 1;
    idx = idx + p.hopsize; 
end

wav = wav * (10^(p.dB_max/20)); %% rescale to dB max (default is 96dB = 2^16)

mel = zeros(c.num_filt, frames);
ceps = zeros(p.num_ceps_coeffs-1+p.use_first_coeff, frames);
if p.visu,
    ffts = zeros(p.fft_size/2+1, frames);
end

w = hann(p.fft_size);

idx = 1:p.fft_size;
for i=1:frames,
    X = abs(fft(wav(idx).*w,p.fft_size)/sum(w)*2).^2;
    mel(:,i) = mfccFilterWeights * X(1:end/2+1); 
    if p.visu,
        ffts(:,i) = X(1:end/2+1);
    end
    idx = idx + p.hopsize;
end

mel(mel<1) = 1; %% for dB
mfcc = DCT*10*log10(mel); %% compute dB and compress using DCT

if p.visu, %% intermediate results of processing steps    
    figure;
    subplot(3,1,1); %% fft
    set(gca,'fontsize',8)
    ffts(ffts<1) = 1; ffts = 10*log10(ffts); imagesc(ffts); set(gca,'ydir','normal','xtick',[])
    title('Spectrum')
    
    subplot(3,1,2); 
    set(gca,'fontsize',8)
    imagesc(mfcc); set(gca,'ydir','normal','xtick',[])
    title('MFCC Representation')
    
    subplot(3,1,3); %% decompressed coefficients
    set(gca,'fontsize',8)
    imagesc(DCT'*mfcc); set(gca,'ydir','normal','xtick',[])    
    title('Reconstructed')
end