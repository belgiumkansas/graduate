function [data] = data_path_extraction(rootdir)
%%
%% extra path and genre data from stored wav files
%%    [data] =data_path_extraction(rootdir)
%%
%% USAGE
%%   data = data_path_extraction(rootdir);  %% normal usage
%%
%% INPUT
%%   rootdir the absolute path too tracks
%%                                   
%% OUTPUT
%%   data struct
%%   data.list_wavs: list of wav file locations
%%   data.class: class of each wav file
%%   data.class_name: name of respective classes
%%


 %% scandir for directories
    d = dir(rootdir);
    d = char(d.name);
    d = d(3:end,:); %% ignore "." and ".."

    dirs = {};
    for i=1:size(d,1);
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
end


