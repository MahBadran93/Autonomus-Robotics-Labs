
function result = VectorPlaneAtInfinity(initParams)

global PPM;
% initial intrinsics
CameraIntrinsics = [initParams(1) initParams(2) initParams(3);
                        0 initParams(4) initParams(5); 0 0 1];
% Image of A. conic
Winv = CameraIntrinsics * CameraIntrinsics'; % 3x3 


% projection matrix
P = PPM(:,:,2);

% initalise n and Create sympolic variables to hold the vector 
%syms x y z

%result = rand(3,1);
syms x1 x2 x3

n =[x1; x2; x3];

% Dual Absolute Quadratic(DAQ).
Q_inf = [ Winv, (Winv * n); (n' * Winv), (n' * Winv * n)];% Winv is known, we need to find n 

% n is null vector of Q_inf, yields this relation: Q_inf.n= 0, Winv = P *  Q_inf * P'

% DAQ eqs, we have 9 equations to solve with 3 unknown x,y,z// Winv = P *  Q_inf * P'
daq = P *  Q_inf * P';
eqn1 = (daq(1,1) - Winv(1,1)) == 0;
eqn2 = (daq(1,2) - Winv(1,2)) ==0;
eqn3 = (daq(1,3)- Winv(1,3))==0;
eqn4 = (daq(2,1) - Winv(2,1))==0; 
eqn5 = (daq(2,2) - Winv(2,2))==0;
eqn6 = (daq(2,3)- Winv(2,3))==0;
eqn7 = (daq(3,1)- Winv(3,1))==0; 
eqn8 = (daq(3,2) - Winv(3,2))==0; 
eqn9 = (daq(3,3) - Winv(3,3))==0;  

% stack the equations in a vector, put 3 equations 
eqns = [eqn1 eqn4 eqn6];

%assume([x~=0,y~=0,z~=0])

% solve the equations and return only the real solution 
sol = vpasolve(eqn1 ,x1,x2,x3);
% return vector of the result converted to double values 
result = [double(sol.x1); double(sol.x2); double(sol.x3)];

%result = [double(sol.x1),double(sol.x2),double(sol.x3)];
%result = sol;
%n = [double(result.x) double(result.y) double(result.z)];
%result = Q_inf;

end


