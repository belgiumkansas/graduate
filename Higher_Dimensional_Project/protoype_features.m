

%one data path per computer do not delete paths comment them out.
data_path = 'C:\Users\Johnny_Five\Google Drive\ECEN5322\Final_project_data\'
%data_path = 'C:\Users\Johnny_Five\Google Drive\ECEN5322\Final_project_data\'

%strcar local data path with the data to be used
x = strcat(data_path, 'Transformed_data\mfcc19_mid_19.mat');

%loads
if( exist('mfcc'))
    disp('mfcc already loaded')
else
    load(x);
end
load('data_catagorization.mat');
%clear loading variables
%clearvars -except mfcc data

mfcc_num_coef = length(mfcc{1}(:,1));
xbins = linspace(-100,100,100);
test = 300;
for i=test:test%length(data.list_wavs)
    
    
   for j=1:mfcc_num_coef
       Z{1}(j,:) = hist(mfcc{i}(j,:),xbins);
   end
   %normalize
   Z{1} = Z{1}./length(mfcc{1}(1,:));
end
colormap('gray');
figure(1);
imagesc(mfcc{i});

colormap('gray');
figure(2)
imagesc(Z{1});


    
%avg = sum(mfcc_mid_19,3)./length(mfcc_mid_19(1,1,:));


    
