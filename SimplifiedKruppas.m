function result = SimplifiedKruppas(initParams)
    global Fs;
    result = [];
    count = 1;
    CameraIntrinsics = [initParams(1) initParams(2) initParams(3);
                        0 initParams(4) initParams(5); 0 0 1];
    Winv = CameraIntrinsics * CameraIntrinsics';
    for i=1:size(Fs,3) 
        for j=i+1:size(Fs,4)   
            if(i==j)
                continue;
            end    
            
            [U, E, V] = svd(Fs(:,:,i,j)');
            
            % get r and s which are the non zero singular values in of Fij
            r = E(1,1);
            s = E(2,2);
        
            % Separate the U elements to apply in the equations.
            u1 = U(:,1);
            u2 = U(:,2);
             % Separate V elements the same    
            v1 = V(:,1);
            v2 = V(:,2);
            % After extracting the needed values we applyit in the eqs
            eq1 = (r^2 * v1' * Winv * v1) / (u2' * Winv * u2);
            eq2 = (r * s * v1' * Winv * v2) / (-u1' * Winv * u2);
            eq3 = (s^2 * v2' * Winv * v2) / (u1' * Winv * u1);
            % Induce the Cost function, as seen in advanced in visual
            % computing 
            %result(count) = sqrt((eq1^2 + eq2^2 + eq3^2)); 
            result(count) = norm(((norm(eq1)) - (norm(eq3)))  ,'fro');
            count = count + 1;
        end

    end
    


end

