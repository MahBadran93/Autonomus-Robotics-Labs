function result = cost_function(initParam)

    global Fs;
    % initial camera intrinsics
    % reshape to 3x3 because the iniParam is sent as 1x4 vector with the
    % values to be optimised
    CameraIntrinsics = [initParam(1) initParam(2) initParam(3); 0 initParam(4) initParam(5); 0 0 1];
    result = 0; % result initalisation

    %CameraIntrinsics = initParam;

    
    % iterate over all the Fs matrices(10x10) to get the two equal singular values,
    % the result of svd function will be save in U E V and the singulr
    % values are in E matrix, the diagonal elemtents so E(1,1) is the first
    % singular value and E(2,2) is the second value 
    for i=1:size(Fs,3) 
        for j=i+1:size(Fs,4)    
            Essintial = CameraIntrinsics' * Fs(:,:,i,j) * CameraIntrinsics;    
            [~,E,~] = svd(Essintial);
            result = (E(1,1) - E(2,2)) / (E(1,1) + E(2,2)); % first cost function 
            %result = (E(1,1) - E(2,2)) / E(2,2); % seocnd cost function 

        end
        
    end
end 