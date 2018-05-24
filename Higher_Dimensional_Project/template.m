

%one data path per computer do not delete paths comment them out.

%data path for pre extracted data
data_path = 'C:\Users\Johnny_Five\Google Drive\ECEN5322\Final_project_data\Transformed_data\';

%root path of music files
rootdir = 'C:\Users\Johnny_Five\Google Drive\ECEN5322\Final_project_data\Music Data\tracks\';

fs = 11025;
data = data_path_extraction(rootdir);
p = create_ma_parameters(fs);


%strcar local data path with the file to be loaded
load(strcat(data_path, 'mfcc20_full.mat'));












    
