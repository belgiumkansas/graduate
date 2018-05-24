%one data path per computer do not delete paths comment them out.

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
    
    load(strcat(data_path, 'gmm_1_mfcc20.mat'));
    % target o/p matrix
    Target = zeros(6,729);
    Target(1,1:320) = 1;
    Target(2,321:434) = 1;
    Target(3,435:460) = 1;
    Target(4,461:505) = 1;
    Target(5,506:607) = 1;
    Target(6,608:729) = 1;
    


    Target_new = zeros(729,2);
    Target_new(1:320,1) = 1;
    Target_new(321:434,1) = 2;
    Target_new(435:460,1) = 3;
    Target_new(461:505,1) = 4;
    Target_new(506:607,1) = 5;
    Target_new(608:729,1) = 6;
    
%     Target_new_2 = zeros(1,729);
    indices = crossvalind('Kfold',Target_new(:,1),5);
    
    Target_new(:,2) = indices;
    indices_t = indices';

for q = 1: 10
        for i = 1:5

            % Validation set is taken as the (i+1)th test and when (i-1) when
            % i=k. (k-fold validation)

            if i<5
                v_i = i+1;
            else
                v_i = i-1;
            end


            gmm_test = gmm_1(indices == i, :);
            gmm_valid = gmm_1(indices == v_i,:);
            gmm_train = gmm_1(indices ~= i & indices ~=v_i,:);

            Intmdt_Target_train = Target_new(indices ~= i & indices ~=v_i);
            Intmdt_Target_valid = Target_new(indices==v_i);
            Intmdt_Target_test = Target_new(indices==i);


            Final_Target_test = zeros(length(Intmdt_Target_test),6);
            Final_Target_valid = zeros(length(Intmdt_Target_valid),6);
            Final_Target_train = zeros(length(Intmdt_Target_train),6);

            for j = 1:6
                Final_Target_test(:,j) = double(Intmdt_Target_test==j);
                Final_Target_valid(:,j) = double(Intmdt_Target_valid==j);
                Final_Target_train(:,j) = double(Intmdt_Target_train==j);
            end


            Target = (vertcat(Final_Target_train,Final_Target_valid,Final_Target_test))';
            gmm = (vertcat(gmm_train,gmm_valid,gmm_test))';

            x = gmm;
            t = Target;

            % Choose a Training Function
            % For a list of all training functions type: help nntrain
            % 'trainlm' is usually fastest.
            % 'trainbr' takes longer but may be better for challenging problems.
            % 'trainscg' uses less memory. Suitable in low memory situations.
            trainFcn = 'trainbr';  % Scaled conjugate gradient backpropagation.

            % Create a Pattern Recognition Network
            hiddenLayerSize = 20;
            net = patternnet(hiddenLayerSize, trainFcn);

            % Setup Division of Data for Training, Validation, Testing
            trainInd = 1:length(Final_Target_train);
            valInd = length(Final_Target_train):length(Final_Target_valid)+length(Final_Target_train);
            testInd = valInd:length(gmm);


            [trainInd,valInd,testInd] = divideind(gmm,trainInd,valInd,testInd);

            % Train the Network
            [net,tr] = train(net,x,t);

            % Test the Network
            y = net(x);
            e = gsubtract(t,y);
            performance = perform(net,t,y);
            tind = vec2ind(t);
            yind = vec2ind(y);
            percentErrors = sum(tind ~= yind)/numel(tind);  

            % Training Confusion Plot Variables
            yTrn = net(x(:,tr.trainInd));
            tTrn = t(:,tr.trainInd);

            % Validation Confusion Plot Variables
            yVal = net(x(:,tr.valInd));
            tVal = t(:,tr.valInd);

            % Test Confusion Plot Variables
            yTst = net(x(:,tr.testInd));
            tTst = t(:,tr.testInd);

            % Overall Confusion Plot Variables
            yAll = net(x);
            tAll = t;

            % Plot Confusion
            figure(i), plotconfusion(tTrn, yTrn, 'Training', tVal, yVal, 'Validation', tTst, yTst, 'Test', tAll, yAll, 'Overall')        

            % View the Network
            % view(net);

            % Plots       
            %figure, plotperform(tr)
            %figure, plottrainstate(tr)
            %figure, ploterrhist(e)
            %figure, plotconfusion(t,y)
            %figure, plotroc(t,y)  

            [R,Result] = max(yTst);  
            for p = 1:length(tTst)
                Original(p) = find(tTst(:,p));
            end
            [C((q-1)*5+i,:,:),order] = confusionmat(Original,Result);


        end
    end




















   