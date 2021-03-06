function result = DacCostFunction(initParams)

   % initial intrinsics
    CameraIntrinsics = [initParams(1) initParams(2) initParams(3);
                        0 initParams(4) initParams(5); 0 0 1];

    planeAtInf = VectorPlaneAtInfinity(initParams);
    Homography = HomographyInf(planeAtInf,CameraIntrinsics );
    result = zeros(size(Homography,2));
    CameraIntrinsic = [initParams(1) initParams(2) initParams(3); 0 initParams(4) initParams(5); 0 0 1];
         
    Winv = CameraIntrinsic * CameraIntrinsic';
    for i = 1:size(Homography,2)
        %ss = norm((Homography{i} * Winv * Homography{i}')-Winv,'fro'); %result(count) = Homography(count); 
        result(i) = norm( result((i+1)) - (Homography{i} * Winv * Homography{i}'),'fro');          
end

