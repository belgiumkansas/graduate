%rootdir = 'C:\Users\jeff\Google Drive\ECEN5322\Final_project_data\Music Data\tracks\';
rootdir = 'C:\Users\Johnny_Five\Google Drive\ECEN5322\Final_project_data\Music Data\tracks\';

% sample rate of all wav files in the directories
fs = 11025; 

%data path for pre extracted data
data_path = 'C:\Users\Johnny_Five\Google Drive\ECEN5322\Final_project_data\Transformed_data\';

data = data_path_extraction(rootdir);
p = create_ma_parameters(fs);

%load mfcc20_similarity
load(strcat(data_path, 'cms_mfcc20_similarity'));

%load fp full distance
%load(strcat(data_path, 'mfcc_full_fp_distance'));
%D = fp_dist;

%set infinite distances too arbitrary large distance
D(isinf(D)) = 10e5;
K_nearest = 10;

for i=7:15
    K_nearest = 5+i;
    C(i) = cross_validation(D, K_nearest, data);
    disp(K_nearest);
end













