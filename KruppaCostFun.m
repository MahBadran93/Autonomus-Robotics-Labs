function result = KruppaCostFun(initParams)
    % input : initParams which is a 1x5 vector of inital intrinsic parameters 

    global Fs

    % Summation Result initalize 
    result = 0

    % Matrix Form of initial intrinsics 
    CameraIntrinsics = [initParams(1) initParams(2) initParams(3); 0 initParams(4) initParams(5); 0 0 1];

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
            
            % Find the epipole e_ij which is the projection of the camera i
            % center  on camera j, and the epipole can be found by
            % extracting the SVD of the fundimental matrix F_ij transposed and we obtain U,E,V 
            % matrices and e_ji is the last column of V transpose
            [~,~,V] = svd(Fs(:,:,i,j)');
            Vtranspose = V';
            % get last column of V transpose which is the epipole
            e_ji = Vtranspose(:,end);
            
            eqs2 = e_ji * Winv * e_ji';
            eqs2Norm = norm(eqs2,'fro');
            
            % Cost function 
            result = result + eqs/eqsNorm - eqs2/eqs2Norm;
            
            
        end

    end



     % initalised output result
     result = 0 
 

end

