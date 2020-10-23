function mTrajectoryPlanner_TF3D(obj,t)

Qda = obj.pPos.Qd;

%% Test Position

% x = 1*-0.25;
% y = 0*0.25;
% z = 1.6;
% 
% p = 2;
% q = 2;
% beta = pi/3;
% 
% phi = -pi; % Roll (x-axis)
% theta = 0; % Pitch (y-axis)
% psi = -0.3218; % Yaw (z-axis)

%% Test Position LOAD

% x = -0.25;
% y = 0.25;
% z = 1.5;
% 
% p = 1.5;
% q = 1.5;
% beta = pi/3;
% 
% phi = -pi; % Roll (x-axis)
% theta = 0; % Pitch (y-axis)
% psi = -0.3218; % Yaw (z-axis)

%% Test Orientation LOAD 

% x = -0.25;
% y = 0;
% z = 1.6;
% 
% p = 1.6;
% q = 1.6;
% beta = pi/3;
% 
% T1 = 5;
% T2 = T1 + 15;
% T3 = T2 + 15;
% T4 = T3 + 15;
% T5 = T4 + 15;

% phi = 0; % Roll (x-axis)
% theta = 0; % Pitch (y-axis)
% psi = -0.1; % Yaw (z-axis)

% phi = 0; % Roll (x-axis)
% theta = 0; % Pitch (y-axis)
% psi = -pi/3; % Yaw (z-axis)
% 
% 
% if t <= T1
%     disp('CASE 1')
%     phi = 0; % Roll (x-axis)
%     theta = pi/4*(t)/(T1); % Pitch (y-axis)
%     psi = -0.1; % Yaw (z-axis)
% 
% elseif t <= T2
%     disp('CASE 2')
%     phi = 0; % Roll (x-axis)
%     theta = 0; % Pitch (y-axis)
%     psi = -0.1 + (-pi/2 + 0.1)*(t-T1)/(T2 - T1); % Yaw (z-axis)
% 
% elseif t <= T3
%     disp('CASE 3')
%     phi = 0; % Roll (x-axis)
%     theta = (pi/4)*(t-T2)/(T3 - T2); % Pitch (y-axis)
%     psi = -pi/2; % Yaw (z-axis)
% 
% end


%% Pringles

% rX = 0.25; % [m]
% rY = 0.25; % [m]
% T = 15;   % [s]
% Tf = 30;
% w = 2*pi/T; % [rad/s]
% 
% t_traj = t;
% a = 3*(t_traj/Tf)^2 - 2*(t_traj/Tf)^3;
% tp = a*Tf;
% 
% x = -0.3 + rX*cos(w*tp);
% y = rY*sin(w*tp);
% z = 1.25 + 0.5*cos(2*w*tp);
% 
% p = 2;
% q = 2;
% beta = pi/3;
% 
% phi = -pi; % Roll (x-axis)
% theta = 0; % Pitch (y-axis)
% psi = -0.3218; % Yaw (z-axis)


%% Carrossel

% % % rX = 0*0.25; % [m]
% % % rY = 0*0.25; % [m]
% % % T = 15;   % [s]
% % % Tf = 30;
% % % w = 2*pi/T; % [rad/s]
% % % 
% % % t_traj = t;
% % % a = 3*(t_traj/Tf)^2 - 2*(t_traj/Tf)^3;
% % % tp = a*Tf;
% % % 
% % % x = -.25 + rX*cos(w*tp);
% % % y = rY*sin(w*tp);
% % % z = 1.5;
% % % 
% % % p = 1.75;
% % % q = 1.75;
% % % beta = pi/3;
% % % 
% % % phi = pi; % Roll (x-axis)
% % % if t > 14.95
% % %     phi = 0;
% % % end
% % % theta = 0; % Pitch (y-axis)
% % % psi = -0.3218 + (-pi + 0.3218)*t/(0.5*Tf); % Yaw (z-axis)

%% Carrossel

rX = 0.25; % [m]
rY = 0.25; % [m]
T = 20;   % [s]
Tf = 40;
w = 2*pi/T; % [rad/s]

t_traj = t;
a = 3*(t_traj/Tf)^2 - 2*(t_traj/Tf)^3;
tp = a*Tf;

x = -.25 + rX*cos(w*tp);
y = rY*sin(w*tp);
z = 1.6;

p = 1.6;
q = 1.6;
beta = pi/3;

phi = 0; % Roll (x-axis)
theta = 0; % Pitch (y-axis)
psi = -0.1 + (-pi + 0.1)*tp/(0.5*Tf); % Yaw (z-axis)

% if t > 30
%     theta = pi/6;
% end


%% 

% rX = 0.25; % [m]
% rY = 0.25; % [m]
% T = 25;   % [s]
% Tf = 50;
% w = 2*pi/T; % [rad/s]
% 
% t_traj = t;
% a = 3*(t_traj/Tf)^2 - 2*(t_traj/Tf)^3;
% tp = a*Tf;
% 
% x = -.3 + rX*cos(w*tp);
% y = rY*sin(w*tp);
% 
% z = 1.6 ;%+ 0.2*cos(2*w*t);
% 
% p = 1.6;
% q = 1.6;
% beta = pi/3;
% 
% phi = 0; % Roll (x-axis)
% theta = 0; % Pitch (y-axis)
% psi = -0.3218 + (-pi + 0.3218)*tp/(Tf); % Yaw (z-axis)


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
% x = (x1d + x2d + x3d)/3;
% y = (y1d + y2d + y3d)/3;
% z = (z1d + z2d + z3d)/3;
% 
% % Shape (S)
% p = sqrt((x1d - x2d)^2 + (y1d-y2d)^2 + (z1d-z2d)^2);
% q = sqrt((x1d - x3d)^2 + (y1d-y3d)^2 + (z1d-z3d)^2);
% r = sqrt((x2d - x3d)^2 + (y2d-y3d)^2 + (z2d-z3d)^2);
% beta = acos((p^2 + q^2 - r^2)/(2*p*q));
% 
% % Orientation (O)
% phi = atan2((2*z1d - z2d - z3d),(2*y1d - y2d - y3d)); % Roll (x-axis)
% theta = -atan((2*z1d - z2d - z3d)/(2*x1d - x2d - x3d)); % Pitch (y-axis)
% psi = atan2((2*y1d - y2d - y3d),(2*x1d - x2d - x3d)); % Yaw (z-axis)

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
