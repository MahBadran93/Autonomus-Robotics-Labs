disp('start Kruppa’s equations')
load('data.mat')
global A
% apply optimisation algorithm with tolerance 
optimisedAlg = optimset('Algorithm','levenberg-marquardt', 'TolX',1e-10,'TolFun',1e-10);
init = [A(1,1) A(1,2) A(1,3) A(2,2) A(2,3)];


intrinsicOptimised = lsqnonlin('KruppaCostFun',init,[],[],optimisedAlg); 

% save the vector values to matrix form 
finalResult(1,1) = intrinsicOptimised(1);
finalResult(1,2) = intrinsicOptimised(2);
finalResult(1,3) = intrinsicOptimised(3);
finalResult(2,1) = 0;
finalResult(2,2) = intrinsicOptimised(4);
finalResult(2,3) = intrinsicOptimised(5);
finalResult(3,1) = 0;
finalResult(3,2) = 0;
finalResult(3,3) = 1;




% Result 
disp('Optimised Intrinsic parameters: ');
disp(finalResult);