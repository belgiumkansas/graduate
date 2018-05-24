
%root where Transformed data can be found
rootdir = 'C:\Users\Johnny_Five\Google Drive\ECEN5322\Final_project_data\Transformed_data\';

% load('data_catagorization');
% load(rootdir, 'ma_cms_mfcc20.mat');

%similarity matrix using 30 gaussian clustering
%and monetcarlo method

if 1
    for i=1:length(data.list_wavs)
        for j=1:length(data.list_wavs)
            D(i,j) = ma_cms(cm_gmm(i),cm_gmm(j),p.ap);
            fprintf('i=%g j=%g\n', i, j);
        end
    end
end