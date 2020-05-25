function result = HomographyInf(vectorOfPlaneInf,init)
    % we can compute homography using (e_jiMatrixForm * Fs + eji * vectorOfPlane')
    global Fs;
    count = 1;
    result = [];
    Winv = init * init';
    Q_inf = [ Winv, (Winv * vectorOfPlaneInf); (vectorOfPlaneInf' * Winv), (vectorOfPlaneInf' * Winv * vectorOfPlaneInf)];
    [U E V] = svd(Q_inf);
    H = diag(E);
    
    for i=1:size(Fs,3) 
        for j=i+1:size(Fs,4)   
            if(i==j)
                continue;
            end  
            % get epipole     
            [~,~,V] = svd(Fs(:,:,i,j)');
            e_ji = V(:,end);
            % epipole to matrix form 
            e_jiMatrixForm = [0,-e_ji(3),e_ji(2); e_ji(3),0,-e_ji(1);-e_ji(1),e_ji(3),0];
            % return 3x3 matrix 
            result{count} = e_jiMatrixForm * (Fs(:,:,i,j) + (e_ji * vectorOfPlaneInf')) ;
            count = count + 1;
        end
    end
    
end

