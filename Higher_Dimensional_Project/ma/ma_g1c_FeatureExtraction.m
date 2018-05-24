function FeatureExtraction(in_file,out_dir)
%%
%%         G1C, implementation as described in thesis
%%         for mirex, "music audio search" (was audio music similarity)
%%
%% USAGE EXAMPLES
%%       FeatureExtraction(in_file,out_dir)
%%       FeatureExtraction('mypath/myInputFile.txt','mypath/myOutputDirectory')
%%       FeatureExtraction('mypath/myInputFile','mypath/myOutputDirectory/')
%%
%% INPUT ARGUMENTS
%%
%% in_file: path fo file containing list of wav files to extract features from
%%          all files are 22050Hz mono wav (this function checks if this is true)
%% out_dir: directory to which this function can write output. (logfiles,
%%          and extracted features), files written to out_dir must be read
%%          by function "DistanceCumputation


%%
%% in_file format (text):
%%      path/to/audiofile/0000001.wav
%%      path/to/audiofile/0000002.wav
%%      path/to/audiofile/0000003.wav
%%      ...
%%      path/to/audiofile/9999999.wav
%%

%% HARDCODED PARAMETERS
exit_when_done = 0; %% set to 1 for final submission (use 0 for testing)
data.submission_name = 'G1C';
FS = 22050; %% all files expected to be 22050 Hz
%%

if nargin~=2,
    error('Number of input arguments is not 2. (try "help FeatureExtraction")')
end

disp('--> G1C (Elias Pampalk, MIREX''06)')

if out_dir(end)~='/' && out_dir(end)~='\',
    out_dir(end+1)='/';
end
output_file = [out_dir,'G1C_features.mat'];

%% TEST WRITE ACCESS TO OUTPUT DIRECTORY
fid = fopen([out_dir,'testwritefile'],'w');
if fid==-1, error('cannot write to output directory'); end
fclose(fid);
delete([out_dir,'testwritefile']);

%% TEST READ ACCESS TO INPUT
fid = fopen(in_file,'r');
if fid==-1, error('cannot read from input file (does it exist?)'); end
fclose(fid);

%% START LOGFILE
logfile = [out_dir,'FeatureExtraction-',data.submission_name,'-logfile.txt'];
fid = fopen(logfile,'a');
if fid==-1, error('can''t append logfile'); end
fclose(fid);

mydisp(logfile,datestr(now));
mydisp(logfile, '-> FeatureExtraction called.')
mydisp(logfile,['   Input file: ',in_file])
mydisp(logfile,['   Output directory: ',out_dir])

%% LOAD INPUT FILE
data.filenames = textread(in_file,'%s','delimiter','\n');
if isempty(data.filenames),
    mydisp(logfile,'-- length(data.filenames)==0');
    mydisp(logfile,'-- something is wrong with the input file, or something went wrong reading from it.');
    error('-- test failed')
end
if isempty(data.filenames{end}), %% ignore empty lines in input file (e.g. at the end of the file)
    tmp = {};
    for i=1:length(data.filenames)-1,
        if ~isempty(data.filenames{i}),
            tmp{end+1} = data.filenames{i};
        end
    end
    data.filenames = tmp;
end
mydisp(logfile,['found ',num2str(length(data.filenames)),' files']);

%% beyond this point an error will cause matlab to exit (if exit_on_error==1)
try
    %% TEST IF WAV FILES EXIST AND ARE IN EXPECTED FORMAT
    if 1,
        mydisp(logfile,'checking if all files are readable 22kHz wav mono ...')
        for i_files=1:length(data.filenames),
            file = data.filenames{i_files};
            fid = fopen(file);
            if fid==-1,
                mydisp(logfile,'-- audio file can''t be opened:');
                mydisp(logfile,['-- ',file]);
                mydisp(logfile,'-- check if path is correct?');
                error('-- test failed');
            end
            fclose(fid);

            if 1,
                [siz,fs] = wavread(file,'size');
                if fs~=FS,
                    mydisp(logfile,['-- wrong sampling rate of file: ', ...
                        file,' is: ',num2str(fs),'and not: ',num2str(FS)]);
                    mydisp(logfile,'use e.g. winamp to check sampling rate?');
                    error('-- test failed')
                end
            end
        end
        mydisp(logfile,'test complete! starting computations ...')
    end

    data.features = zeros(length(data.filenames),1);

    if 1, %% CONSTANTS FOR FEATURE EXTRACTION
        num_ceps_coeffs = 20;
        c.fs = 22050;
        c.num_filt = 36; %% number of Mel frequency bands
        c.seg_size = 512; %% 23ms if c.fs == 22050Hz
        c.hop_size = 512;

        f = linspace(0,c.fs/2,c.seg_size/2+1); %% frequency bins of P
        mel = log(1+f/700)*1127.01048;
        mel_idx = linspace(0,mel(end),c.num_filt+2);

        f_idx = zeros(c.num_filt+2,1);
        for i=1:c.num_filt+2,
            [tmp f_idx(i)] = min(abs(mel - mel_idx(i)));
        end
        freqs = f(f_idx);

        %% height of triangles
        h = 2./(freqs(3:c.num_filt+2)-freqs(1:c.num_filt));

        c.mel_filter = zeros(c.num_filt,c.seg_size/2+1);
        for i=1:c.num_filt,
            c.mel_filter(i,:) = ...
                (f > freqs(i) & f <= freqs(i+1)).* ...
                h(i).*(f-freqs(i))/(freqs(i+1)-freqs(i)) + ...
                (f > freqs(i+1) & f < freqs(i+2)).* ...
                h(i).*(freqs(i+2)-f)/(freqs(i+2)-freqs(i+1));
        end

        c.DCT = 1/sqrt(c.num_filt/2) * ...
            cos((0:num_ceps_coeffs-1)'*(0.5:c.num_filt)*pi/c.num_filt);
        c.DCT(1,:) = c.DCT(1,:)*sqrt(2)/2;
        c.w = 0.5*(1-cos(2*pi*(0:c.seg_size-1)/(c.seg_size-1)))';

        %% FP constants
        c.trans = [1 2 3 3 4 4 5 5 6 6 7 7 8 8 9 9 9 9 ...
            10 10 10 10 10 11 11 11 11 11 11 ...
            12 12 12 12 12 12 12];
        c.f = 22050/512/2*(0:64)/64;
        c.flux_w = repmat(1./(c.f(2:32)/4+4./c.f(2:32)),12,1);  %% see zwicker book p. 254
        c.gauss_w = [.05 .1 .25 .5 1 .5 .25 .1 .05];
        c.blur = filter2(c.gauss_w,eye(12));
        c.blur = c.blur./repmat(sum(c.blur,2),1,12);
        c.blur2 = filter2(c.gauss_w,eye(30));
        c.blur2 = (c.blur2./repmat(sum(c.blur2,2),1,30))';

        %% init feat
        o = zeros(length(data.filenames),1);
        data.feat.fp  = zeros(length(data.filenames),12*30);
        data.feat.fpg = o;
        data.feat.fp_bass = o;     %% low freq band & modulation > 1Hz
        data.feat.g1.m   = zeros(length(data.filenames),19);
        data.feat.g1.co  = zeros(length(data.filenames),19,19);
        data.feat.g1.ico = zeros(length(data.filenames),19,19);
        data.feat.g1c.max_ico = o;
    end

    t0 = cputime;
    for i_files=1:length(data.filenames),
        t1 = cputime;
        if i_files>1,
            time.avg_t_sofar = (t1-t0)/(i_files-1);
            time.est_tot = (t1-t0)/(i_files-1)*length(data.filenames);
            time.est_rem = time.est_tot - (t1-t0);
            mydisp(logfile,[num2str(i_files),'/',num2str(length(data.filenames)), ...
                ' FE est rem ',num2str(time.est_rem/60),'m, est tot ', ...
                num2str(time.est_tot/60),'m'])
        end

        %% START FEATURE EXTRACTION CODE
        siz = wavread(data.filenames{i_files},'size'); %% use (at most) 120 sec from center
        if siz(1)>FS*120,
            x0 = ceil(siz(1)/2-FS*60);
            x1 = floor(siz(1)/2+FS*60);
        else %% use what ever is available
            x0 = 1;
            x1 = siz(1);
        end
        wav = wavread(data.filenames{i_files},[x0 x1]);
        wav = wav * (10^(96/20));
        
        if 1, %% compute P
            num_segments = floor((length(wav)-c.seg_size)/c.hop_size)+1;
            P = zeros(c.seg_size/2+1,num_segments); %% allocate memory

            for i_p = 1:num_segments,
                idx = (1:c.seg_size)+(i_p-1)*c.hop_size;
                x = abs(fft(wav(idx).*c.w)/sum(c.w)*2).^2;
                P(:,i_p) = x(1:end/2+1);
            end
        end

        if 1, %% compute M dB
            %num_segments = size(P,2);
            M = zeros(c.num_filt,num_segments);

            for i_m = 1:num_segments,
                M(:,i_m) = c.mel_filter*P(:,i_m);
            end

            M(M<1)=1; M = 10*log10(M);
        end

        if 1, %% compute FP
            sone = zeros(12,size(M,2));
            for i_sone=1:12,
                sone(i_sone,:) = sum(M(c.trans==i_sone,:),1);
            end

            if size(sone,2)<128, %% pad with zeros (dont need this because MIREX files are all longer than 30sec)
                tmp_missing = 128-size(sone,2);
                sone = [zeros(size(sone,1),ceil(tmp_missing/2)), ...
                    sone,zeros(size(sone,1),ceil(tmp_missing/2))];
            end

            num_fp_frames = floor((size(sone,2)-128)/64)+1;
            fp = zeros(num_fp_frames,12*30);

            idx = 1:128;
            for k=1:num_fp_frames,
                wsone = sone(:,idx);

                X = fft(wsone,128,2);
                X2 = abs(X(:,2:32)).*c.flux_w; %% amplitude spectrum (not power) 
                X3 = c.blur*abs(diff(X2,1,2))*c.blur2;

                fp(k,:) = X3(:)';
                idx = idx + 64;
            end

            data.feat.fp(i_files,:) = median(fp,1);
        end

        tmp = reshape(data.feat.fp(i_files,:),12,30);
        data.feat.fpg(i_files) = sum(sum(tmp,1).*(1:30))/max(sum(tmp(:)),eps);
        data.feat.fp_bass(i_files) = sum(sum(tmp(1:2,3:30))); %% modulation > 1Hz

        mfcc = c.DCT * M;
        mfcc = mfcc(2:20,:);
        data.feat.g1.m(i_files,:) = mean(mfcc,2);
        data.feat.g1.co(i_files,:,:) = cov(mfcc');
        data.feat.g1.ico(i_files,:,:) = inv(squeeze(data.feat.g1.co(i_files,:,:)));
        data.feat.g1c.max_ico(i_files) = max(data.feat.g1.ico(i_files,:));
    end
    tot_time_FE = cputime-t0;
    mydisp(logfile,['Total CPU Time [h]: ',num2str(tot_time_FE/60/60)])

    mydisp(logfile,['saving data to output file: ',output_file])
    a = version;
    if a(1)=='7', %% matlab 7 has default compression
        mydisp(logfile,'   Matlab 7')
        save(output_file,'data','-v6'); %% dont use compression
    else
        mydisp(logfile,'   Not Matlab 7')
        save(output_file,'data');
    end
    mydisp(logfile,'done. exiting ...')
    mydisp(logfile,datestr(now))
    if exit_when_done,
        exit;
    end
catch
    mydisp(logfile,'-- caught error!')
    mydisp(logfile,['failed at song number: ',num2str(i_files)])
    mydisp(logfile,['          song name:   ',data.filenames{i_files}])
    mydisp(logfile,lasterr)
    mydisp(logfile,datestr(now))
    if exit_when_done,
        exit;
    else
        error('-- error')
    end
end

function mydisp(logfile,str) %% appends logfile
fid = fopen(logfile,'a'); disp(str); fprintf(fid,'%s\r\n',str); fclose(fid);
