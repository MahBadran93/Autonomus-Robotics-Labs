function result = MendoncaCipollaCostfunction(initParam)

    global Fs;
    
    % reshape to 3x3 because the iniParam is sent as 1x5 vector with the
    % values to be optimised
    CameraIntrinsics = [initParam(1) initParam(2) initParam(3);
        0 initParam(4) initParam(5); 0 0 1];
    
    result = []; % initalise vector with number of fund. matrices
    
    count = 1;
    
    % iterate over all the Fs matrices(10x10) to get the two equal singular values,
    % the result of svd function will be save in (~ E ~) and the singulr
    % values are in E matrix, the diagonal elemtents so E(1,1) is the first
    % singular value and E(2,2) is the second value 
    for i=1:size(Fs,3) 
        for j=i+1:size(Fs,4)   
            if i == j
                continue; % pass over zero values fund. matrices
            end    
            Essintial = CameraIntrinsics' * Fs(:,:,i,j) * CameraIntrinsics;    
            [~,E,~] = svd(Essintial);
            %result(count) = ((E(1,1) - E(2,2)) / (E(1,1)) + E(2,2)); % first cost function 
            result(count)  = ((E(1,1) - E(2,2)) / E(2,2)); % seocnd cost function 
            count = count + 1;
         
        end
    end
end 