function mTrajectoryPlanner_TF3D_mexido(obj,t)

Qda = obj.pPos.Qd;

rX = 0.5; % [m]
rY = 0.5; % [m]
T = 15;   % [s]
Tf = 30;
w = 2*pi/T; % [rad/s]

t_traj = t;
a = 3*(t_traj/Tf)^2 - 2*(t_traj/Tf)^3;
tp = a*Tf;

% --------------------------------------------------------------------
% Position (P)
x = -.25;
y = 0;
z = 1.5;

% x = -.3 + rX*sin(2*w*tp);
% y = rY*sin(w*tp);
% z = 1.5;

% x = rX*cos(w*tp);
% y = rY*sin(w*tp);
% z = 1.25 + 0.5*cos(2*w*tp);

% --------------------------------------------------------------------
% Shape (S)
p = 2;
q = 2;
% r = 1;
beta = pi/3;

% --------------------------------------------------------------------
% Orientation (O)
phi = -pi; % Roll (x-axis)
theta = 0; % Pitch (y-axis)
psi = -0.3218 + (pi + 0.3218)*t/(0.5*Tf); % Yaw (z-axis)   -0.3218; 
% psi = 0;%-0.3218; % Yaw (z-axis)

% phi = pi/2; % Roll (x-axis)
% theta = pi/6; % Pitch (y-axis)
% psi = pi/3; % Yaw (z-axis)

%% Inverse Transformation

r = sqrt(p^2 + q^2 - 2*p*q*cos(beta));
h1 = sqrt(0.5*(p^2 + q^2 - 0.5*r^2));
h2 = sqrt(0.5*(r^2 + p^2 - 0.5*q^2));
h3 = sqrt(0.5*(q^2 + r^2 - 0.5*p^2));
alpha1 = acos((4*(h1^2 + h2^2) - 9*p^2)/(8*h1*h2));
alpha2 = acos((4*(h1^2 + h3^2) - 9*q^2)/(8*h1*h3));

% Robot 1 (R1)
x1d = 2/3*h1*cos(theta)*cos(psi) + x;
y1d = 2/3*h1*cos(theta)*sin(psi) + y;
z1d = - 2/3*h1*sin(theta) + z;

% Robot 2 (R2)
x2d =  2/3*h2*cos(alpha1)*cos(theta)*cos(psi) ...
    + 2/3*h2*sin(alpha1)*(sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi)) ...
    + x;
y2d =  2/3*h2*cos(alpha1)*cos(theta)*sin(psi) ...
    + 2/3*h2*sin(alpha1)*(sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi)) ...
    + y;
z2d = - 2/3*h2*cos(alpha1)*sin(theta) ...
    + 2/3*h2*sin(alpha1)*sin(phi)*cos(theta)...
    + z;

% Robot 3 (R3)
x3d =  2/3*h3*cos(alpha2)*cos(theta)*cos(psi) ...
    - 2/3*h3*sin(alpha2)*(sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi)) ...
    + x;
y3d =   2/3*h3*cos(alpha2)*cos(theta)*sin(psi) ...
    - 2/3*h3*sin(alpha2)*(sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi)) ...
    + y;
z3d = - 2/3*h3*cos(alpha2)*sin(theta) ...
    - 2/3*h3*sin(alpha2)*sin(phi)*cos(theta)...
    + z;

% Position (P)
x = (x1d + x2d + x3d)/3;
y = (y1d + y2d + y3d)/3;
z = (z1d + z2d + z3d)/3;

% Shape (S)
p = sqrt((x1d - x2d)^2 + (y1d-y2d)^2 + (z1d-z2d)^2);
q = sqrt((x1d - x3d)^2 + (y1d-y3d)^2 + (z1d-z3d)^2);
r = sqrt((x2d - x3d)^2 + (y2d-y3d)^2 + (z2d-z3d)^2);
beta = acos((p^2 + q^2 - r^2)/(2*p*q));

% Orientation (O)
phi = atan2((2*z1d - z2d - z3d),(2*y1d - y2d - y3d)); % Roll (x-axis)
theta = -atan((2*z1d - z2d - z3d)/(2*x1d - x2d - x3d)); % Pitch (y-axis)
psi = atan2((2*y1d - y2d - y3d),(2*x1d - x2d - x3d)); % Yaw (z-axis)
    
A1A2 = [x2d-x1d y2d-y1d z2d-z1d]';
H1A1 = -[(x2d+x3d)/2-x1d (y2d+y3d)/2-y1d (z2d+z3d)/2-z1d]';
 
% N = cross(A1H1,A1A2);
    
% thetaf = atan2(norm(N(1:2)),N(3));
% psif = pi - atan2(N(2),N(1));
% psif = atan2(N(2),N(1));

psi = atan2(H1A1(2),H1A1(1));
theta = -atan2(H1A1(3),(norm(H1A1(1:2))));

% Matrizes de rotação
  
% Rotação em Y
Ry = [cos(theta)  0 sin(theta);
      0            1 0;
      -sin(theta) 0 cos(theta)];
  
% Rotação em Z
Rz = [cos(psi) -sin(psi) 0;
      sin(psi) cos(psi)  0;
      0         0          1];
  
% Rotação em Y,X e Z nesta ordem
A1A2 = Rz\A1A2;
A1A2 = Ry\A1A2;

H1A1 = Rz\H1A1;
H1A1 = Ry\H1A1;

N = cross(A1A2,H1A1);

phi = atan2(N(3),N(2))+pi/2; 

%% Applying the results

% Formation
% P
obj.pPos.Qd(1) = x;
obj.pPos.Qd(2) = y;
obj.pPos.Qd(3) = z;

% S
obj.pPos.Qd(4) = p;
obj.pPos.Qd(5) = q;
obj.pPos.Qd(6) = beta;

% O
obj.pPos.Qd(7) = phi;
obj.pPos.Qd(8) = theta;
obj.pPos.Qd(9) = psi;

obj.pPos.dQd = (obj.pPos.Qd - Qda)/obj.pPar.Ts;

% Robots
% R1
obj.pPos.Xd(1) = x1d;
obj.pPos.Xd(2) = y1d;
obj.pPos.Xd(3) = z1d;

% R2
obj.pPos.Xd(4) = x2d;
obj.pPos.Xd(5) = y2d;
obj.pPos.Xd(6) = z2d;

% R3
obj.pPos.Xd(7) = x3d;
obj.pPos.Xd(8) = y3d;
obj.pPos.Xd(9) = z3d;

end
