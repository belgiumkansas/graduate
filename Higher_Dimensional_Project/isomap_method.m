%one data path per computer do not delete paths comment them out.

%data path for pre extracted data
data_path = 'C:\Users\Johnny_Five\Google Drive\ECEN5322\Final_project_data\Transformed_data\';

%root path of music files
rootdir = 'C:\Users\Johnny_Five\Google Drive\ECEN5322\Final_project_data\Music Data\tracks\';

fs = 11025;
data = data_path_extraction(rootdir);
p = create_ma_parameters(fs);


%strcar local data path with the file to be loaded
load(strcat(data_path, 'preprocessed\mfcc19_mid_19'));
for i = 1:length(mfcc_mid19)
    mfcc_mid_19_vect(i,:) = reshape(mfcc_mid_19(i), 
end
no_dims = 2;
k = 12;

D(isinf(D)) = 10E5;
if 1
    [mappedX, mapping] = isomap(gmm5_vectors, no_dims, k);
end







    
