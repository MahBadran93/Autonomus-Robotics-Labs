function result = KruppaCostFun(initParams)
    % input : initParams which is a 1x5 vector of inital intrinsic parameters 

    global Fs

    % Summation Result initalize 
    result = 0

    % Matrix Form of initial intrinsics 
    CameraIntrinsics = [initParam(1) initParam(2) initParam(3); 0 initParam(4) initParam(5); 0 0 1];

    %image of absolute conics(conic of points in the image plane) depends on the intrinsic parameters of the
    %camera which allows to be recovered 
    w = inv(CameraIntrinsics)' * inv(CameraIntrinsics);
 
    % conic of lines in the image plane is represented by inverese of w and
    % the intrinsic parameters can be found directly 
    Winv = CameraIntrinsics * CameraIntrinsics';
 
 
    % Apply the cost function to solve Kruppas equation
    for i=1:size(Fs,3) 
        for j=i+1:size(Fs,4)    
            % Apply Classical Kruppa's equations which can be donated by Fij * Winv * 
            % Fij'
            eqs = Fs(:,:,i,j) * Winv * Fs(:,:,i,j)';  

            % find the F.norm of eqs 
            eqsNorm = norm(eqs,'fro');

            [~,E,~] = svd(Essintial);
            result = (E(1,1) - E(2,2)) / (E(1,1) + E(2,2)); % first cost function 
            %result = (E(1,1) - E(2,2)) / E(2,2); % seocnd cost function 

        end

    end



     % initalised output result
     result = 0 
 

end

