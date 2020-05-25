%load the data 
load('data.mat');

disp('AutoCalibration')
global A;
format long g;

% initial intrinsic parameters
init = [A(1,1) A(1,2) A(1,3) A(2,2) A(2,3)];

%................. Methods...........................................

%MendoncaCipolla Method 
%{
optimisedAlg = optimset('Algorithm','levenberg-marquardt','TolX',1e-10);
intrinsicOptimised = lsqnonlin('MendoncaCipollaCostfunction',init,[],[],optimisedAlg);
%}

%Kruppa's Classical
%{
optimisedAlg = optimset('Algorithm','levenberg-marquardt','TolX',1e-10,'TolFun',1e-10 );
intrinsicOptimised = lsqnonlin('KruppaCostFun',init,[],[],optimisedAlg); 
%}

% Simplified Method 
%{
optimisedAlg = optimset('Algorithm','levenberg-marquardt','TolX',1e-20, 'TolFun',1e-12 );
intrinsicOptimised = lsqnonlin('SimplifiedKruppas',init,[],[],optimisedAlg); 
%}

%QAD Method 
optimisedAlg = optimset('Algorithm','levenberg-marquardt','TolX',1e-10);
intrinsicOptimised = lsqnonlin('DacCostFunction',init,[],[], optimisedAlg);
%intrinsicOptimised = lsqnonlin('VectorPlaneAtInfinity',init,[],[], optimisedAlg);


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
