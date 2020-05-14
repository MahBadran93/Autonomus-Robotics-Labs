function result = KruppaCostFun(initParams)
    % input : initParams which is a 1x5 vector of inital intrinsic parameters 

    global Fs

    % Summation Result initalize 
    result = ones(5,1);

    % Matrix Form of initial intrinsics 
    CameraIntrinsics = [initParams(1) initParams(2) initParams(3); 0 initParams(4) initParams(5); 0 0 1];

    %image of absolute conics(conic of points in the image plane) depends on the intrinsic parameters of the
    %camera which allows to be recovered 
    %w = inv(CameraIntrinsics)' * inv(CameraIntrinsics);
 
    % conic of lines in the image plane is represented by inverese of w and
    % the intrinsic parameters can be found directly 
    Winv = CameraIntrinsics * CameraIntrinsics';
 
 
    % Apply the cost function to solve Kruppas equation
    for i=1:size(Fs,3) 
        for j=i+1:size(Fs,4)   
            if(i==j)
                continue;
            end    
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
            e_ji = V(:,end);
            
            % conver ep_ji to skew matrix form to allow cross product 
            e_jiMatrixForm = [0,-e_ji(3),e_ji(2); e_ji(3),0,-e_ji(1);-e_ji(2),e_ji(1),0];
            
            eqs2 = e_jiMatrixForm * Winv * e_jiMatrixForm';
            eqs2Norm = norm(eqs2,'fro');
            
            % Cost function Classical
            result = (eqs/eqsNorm - eqs2/eqs2Norm);
            disp(size(result));
            
            
        end

    end

 

end

