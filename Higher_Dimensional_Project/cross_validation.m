function [ C ] = cross_validation( D, K_nearest, data )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here


fold =5;
%C = cross_validation( D, data, fold);
if 1
    clearvars C_cell
    for q=1:10
        genre_vector = data.class;
        %random vector for breaking up data
        X = fold*rand(length(genre_vector),1);
        %break random vector into 'fold' equal parts
        for i = 1:5
            X((i-1)<X & X<=i)= i;
        end
        %X(2)
        %data structure with lists of what track belong to what group
        clearvars fold_data
        A = ones(5,1);
        for i = 1:length(genre_vector)
            partition = X(i);
            fold_data{partition}(1,A(partition)) = i;
            A(partition) = A(partition)+1;
        end

        %cross validate along each fold
        for i=1:fold
            clearvars D_current_fold G_current_fold G_testing_fold testing_tracks genre Conf
            testing_tracks = fold_data{i};

            %keep distances for non testing tracks
            D_current_fold = D;
            D_current_fold(:,testing_tracks) = []; 
            G_current_fold = genre_vector;
            G_current_fold(:,testing_tracks) = [];
            D_current_fold = D_current_fold(testing_tracks, :);
            G_testing_fold = genre_vector(:,testing_tracks);

            %find nearest neighbors 
            for j=1:length(testing_tracks)
                clearvars nn nn_genres
                %%sort by nearest neighbors
                [B, I] = sort(D_current_fold(j,:));
                nn = I(1:K_nearest);
                for l=1:K_nearest
                    %vote by genres of nearest neighbors
                    nn_genres(l) = G_current_fold(nn(l));
                    genre(j) = mode(nn_genres);
                end    
            end
            %create confusion matrix
            Conf = zeros(6,6);
            Conf = confusionmat(G_testing_fold, genre);
           
            C_total((q-1)*5+i) = trace(Conf)/sum(sum(Conf));
            C_cell((q-1)*5+i,:,:) = Conf./sum(Conf,1);


        end

    end
end

test = nanmean(C_cell, 1);
test = reshape(test,[6,6]);
C.mean = test*100;
test2 = nanstd(C_cell,1,1);
test2 = reshape(test2,[6,6]);
C.std = test2*100;
C.avgmean = mean(C_total);
C.avgstd = std(C_total);


clearvars -except C
%clearvars -except C_total test test2 C_cell fold_data C D fold testing_tracks D_current_fold  G_current_fold G_testing_fold data genre G C_prcnt






end

