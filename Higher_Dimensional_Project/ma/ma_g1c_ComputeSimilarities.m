function ComputeSimilarities(in_dir,out_file)
%%
%%         G1C, implementation as described in thesis
%%         for mirex, "music audio search" (was audio music similarity)
%%
%% USAGE EXAMPLE
%%       FeatureExtraction(in_file,out_dir)
%%       ComputeSimilarities(out_dir,'somepath/distance_matrix.txt')
%%
%% INPUT ARGUMENTS
%%
%% in_dir:   directory to which function "FeatureExtraction" was writing to.
%%           a log file will be created in this directory
%% out_file: whole distance matrix in the following format
%%
%%   <start file, exclude this line>
%%     <SUBMISSION_NAME>
%%     1   path\file1.wav
%%     2   path\file2.wav
%%     ...
%%     N  path\fileN.wav
%%     Q\R   1       2      ...      N
%%     1     0.000  10.234  ...   123.32
%%     2    10.234   0.000  ...    23.45
%%     .     ...     ...    0.000   ...
%%     N     4.1234  6.345  ...     0.0
%%   <end file, exclude this line>
%%
%%   delimiter: tabulator space, number format: float

%% HARDCODED PARAMETERS
exit_on_error = 0; %% set to 1 for final submission (use 0 for testing)
data.submission_name = 'G1C';
%% 

if nargin~=2,
    error('Number of input arguments is not 2. (try "help ComputeSimilarities")')
end

disp('--> G1C (Elias Pampalk, MIREX''06)')

if in_dir(end)~='/' && in_dir(end)~='\',
    in_dir(end+1)='/';
end
in_file = [in_dir,'G1C_features.mat'];

%% TEST WRITE ACCESS TO OUTPUT FILE
fid = fopen(out_file,'w');
if fid==-1, error('cannot write to output file (distance matrix)'); end
fprintf(fid,'%s','testwrite'); fclose(fid);              
delete(out_file);

%% TEST INPUT FILE
fid = fopen(in_file,'r');
if fid==-1, error('cannot find file with extracted features (path problem?)'); end
fclose(fid);
load(in_file)

%% START LOGFILE
logfile = [in_dir,'ComputeSimilarities-',data.submission_name,'-logfile.txt'];
fid = fopen(logfile,'a');
if fid==-1, error('can''t append logfile'); end
fclose(fid);              

mydisp(logfile,datestr(now));
mydisp(logfile, '-> ComputeSimilarities called.')
mydisp(logfile,['   Input directory: ',in_dir])
mydisp(logfile,'         (writing log-file to input directory)')
mydisp(logfile,['   Output file: ',out_file])

try %% big try catch to catch every error, write it to logfile and exit
    mydisp(logfile,'start computing distances ...')

    D = zeros(length(data.filenames),length(data.filenames));
    t0 = cputime;
    t1 = cputime;
    num_computations = length(data.filenames)*(length(data.filenames)-1)/2;
    num_computations_sofar = 0;
    for i=1:length(data.filenames)-1,
        t2 = cputime;
        if t2-t1>2, %% output current status only every 5 seconds (cputime)
            tmp_avg_t_sofar = (t1-t0)/(num_computations_sofar-1);
            tmp_est_tot = (t1-t0)/(num_computations_sofar-1)*num_computations;
            tmp_est_rem = tmp_est_tot - (t1-t0);
            mydisp(logfile,[num2str(num_computations_sofar),'/',num2str(num_computations), ...
                ' CS est rem ',num2str(tmp_est_rem/60),'m, est tot ', ...
                num2str(tmp_est_tot/60),'m'])
            t1 = cputime;
        end
        for j=i+1:length(data.filenames),
            d_g1_computed = false;
            if all([data.feat.g1c.max_ico(i),data.feat.g1c.max_ico(j)]<10^10),
                tmp = squeeze(data.feat.g1.m(i,:,:))-squeeze(data.feat.g1.m(j,:,:));
                d_g1 = ... %% kl distance
                    trace(squeeze(data.feat.g1.co(i,:,:))*squeeze(data.feat.g1.ico(j,:,:))) + ...
                    trace(squeeze(data.feat.g1.co(j,:,:))*squeeze(data.feat.g1.ico(i,:,:))) + ...
                    trace((squeeze(data.feat.g1.ico(i,:,:))+squeeze(data.feat.g1.ico(j,:,:)))*tmp'*tmp);
                d_g1_computed = true;
            end
            d_fp = norm(data.feat.fp(i,:)-data.feat.fp(j,:));
            d_fpg = abs(data.feat.fpg(i)-data.feat.fpg(j));
            d_fpb = abs(data.feat.fp_bass(i)-data.feat.fp_bass(j));

            if d_g1_computed,
                D(i,j) = ...
                    0.7*(-exp(-1/450*d_g1)+0.7950)/0.1493 + ...
                    0.1*(d_fp-1688.4)/878.23 + ...
                    0.1*(d_fpg-1.2745)/1.1245 + ...
                    0.1*(d_fpb-1064.8)/932.79 + ...
                    10; %% ensure all values larger than one (+1.5 should be enough in most cases)
            else %% not evaluated work around to deal with inv covariance problems
                D(i,j) = ...
                    0.4*(d_fp-1688.4)/878.23 + ...
                    0.3*(d_fpg-1.2745)/1.1245 + ...
                    0.3*(d_fpb-1064.8)/932.79 + ...
                    10;
            end
            num_computations_sofar = num_computations_sofar+1;
        end        
    end
    D = D+D';
    disp('done. start writing output ...')

    fid = fopen(out_file,'w');
    fprintf(fid,'%s\r\n',data.submission_name);
    for i=1:length(data.filenames),
        fprintf(fid,'%d\t%s\r\n',i,data.filenames{i});
    end
    fprintf(fid,'%s','Q\R');
    fprintf(fid,'\t%d',1:length(data.filenames));
    fprintf(fid,'\r\n');
    for i=1:length(data.filenames),
        fprintf(fid,'%d',i);
        fprintf(fid,'\t%d',D(i,:));
        fprintf(fid,'\r\n');
    end
    fclose(fid);    

    disp('output file created.')

    tot_time = cputime-t0;
    disp(['total CPU time [h] ',num2str(tot_time/60/60)])
    
    mydisp(logfile,'done. exiting ...')
    mydisp(logfile,datestr(now))
    if exit_on_error, 
        exit; 
    end
catch
    mydisp(logfile,'-- caught error!')
    mydisp(logfile,lasterr)
    mydisp(logfile,datestr(now))
    if exit_on_error, 
        exit; 
    else
        error('-- error')
    end    
end

function mydisp(logfile,str)
fid = fopen(logfile,'a'); disp(str); fprintf(fid,'%s\r\n',str); fclose(fid);
