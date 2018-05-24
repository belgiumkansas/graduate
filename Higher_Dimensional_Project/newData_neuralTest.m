%%testing on new data
clear all

if 1
    
    %data path for pre extracted data
    data_path = 'C:\Users\hj\Google Drive\Final_project_data\Transformed_data\';

    %root path of music files
    rootdir = 'C:\Users\hj\Google Drive\Final_project_data\Music Data\tracks\';

    fs = 11025;
    data = data_path_extraction(rootdir);
    p = create_ma_parameters(fs);


    %strcar local data path with the file to be loaded
%     load(strcat(data_path, 'gmm_1_mfcc20.mat'));
end

if 1
    
    load(strcat(data_path, 'dimension_reduction\gmm_1_mfcc20.mat'));
    % target o/p matrix
    Target = zeros(6,729);
    Target(1,1:320) = 1;
    Target(2,321:434) = 1;
    Target(3,435:460) = 1;
    Target(4,461:505) = 1;
    Target(5,506:607) = 1;
    Target(6,608:729) = 1;
    
    gmm_1 = gmm_1';
    
    x = gmm_1;
    t = Target;

    % Choose a Training Function
    % For a list of all training functions type: help nntrain
    % 'trainlm' is usually fastest.
    % 'trainbr' takes longer but may be better for challenging problems.
    % 'trainscg' uses less memory. Suitable in low memory situations.
    trainFcn = 'trainbr';  % Scaled conjugate gradient backpropagation.

    % Create a Pattern Recognition Network
    hiddenLayerSize = [22 13];
    net = patternnet(hiddenLayerSize, trainFcn);
   % net.numLayers = 3;
    % Setup Division of Data for Training, Validation, Testing
    net.divideParam.trainRatio = 80/100;
    net.divideParam.valRatio = 20/100;
    net.divideParam.testRatio = 0/100;

    % Train the Network
    [net,tr] = train(net,x,t);

    % Test the Network
    y = net(x);
    e = gsubtract(t,y);
    performance = perform(net,t,y);
    tind = vec2ind(t);
    yind = vec2ind(y);
    percentErrors = sum(tind ~= yind)/numel(tind);

    % View the Network
    view(net)

    % Plots
    % Uncomment these lines to enable various plots.
    %figure, plotperform(tr)
    %figure, plottrainstate(tr)
    %figure, ploterrhist(e)
    figure, plotconfusion(t,y)
    %figure, plotroc(t,y)
    
    
end

if 1
    load(strcat(data_path, 'dimension_reduction\test_gmm1_vect.mat'));
    
    ynew = net(test_gmm1_vect');
    [Rnew,Resultnew] = max(ynew);
    
end
