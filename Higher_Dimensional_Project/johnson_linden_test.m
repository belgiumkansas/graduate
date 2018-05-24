clear

load('data.mat');
%load('Final_project_data/Transformed_data/mfcc_mid_19.mat');

training_percent = .9;
set_size = 729;
sample_size = 10;

for Z = 1:500
   
    X = rand(100,1);  
    X = floor(X * 816)+1;

    for i=1:length(data.list_wavs)
        
        D(i,:) = zeros(1,sample_size*19);
        
        for j=1:sample_size
            Q = mfcc_mid_19(i,:,X(j));
            D(i,(j-1)*19 + 1:(j*19)) = Q;
        end
    end


    TS = rand(set_size,1);
    a = 1;
    b = 1;
    %divided data into sample set and training set
    for i=1:set_size
        if(TS(i) <= training_percent)
            T(a,:) = D(i,:);
            T_G(a,1) = data.class(i);
            a = a+1;
        else
            S(b,:) = D(i,:);
            S_G(b,1) = data.class(i);
            b = b + 1;
        end     
    end

    class = knnclassify(S, T, T_G);


    correct = 0;
    for i=1:length(class)
        if(class(i) == S_G(i))
            correct = correct + 1;
        end
    end

    percent(Z,1) = correct/length(class) * 100;
    
    clearvars -except data mfcc_mid_19 set_size percent training_percent q sample_size
    
end

figure(1)
histogram(percent);
xlabel('percent correct');
ylabel('count');










