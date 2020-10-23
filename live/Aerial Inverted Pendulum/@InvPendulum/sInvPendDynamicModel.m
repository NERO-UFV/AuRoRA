function sInvPendDynamicModel(p)
%sInvPendDynamicModel Dynamic model for inverted pendulum on quadrotor
%   Simulating the dynamic model calculated in UAVwInvertedPendulum.m

%% State variables:
%__________________________________________________________________________
%                   1  2  3  4   5   6    7  8  9  10  11  12
% d.pPos.X(1:12) = [x  y  z  ph  th  ps : dx dy dz dph dth dps]
%--------------------------------------------------------------------------
%                   1   2   3   4    5    6     7   8   9   10   11   12  
% d.pPos.dX(1:12)= [dx  dy  dz  dph  dth  dps : ddx ddy ddz ddph ddth ddps]
%==========================================================================
%                   1      2      3      4
% p.pPos.X(1:4)  = [alpha  beta : dalpha dbeta]
%--------------------------------------------------------------------------
%                   1      2       3       4
% p.pPos.dX(1:4) = [dalpha dbeta : ddalpha ddbeta]
%==========================================================================
%      1     2    3   4     5   6 7 8
% q = [alpha beta phi theta psi x y z]
%__________________________________________________________________________
% Constants:
% Gravity -> d.pPar.g = p.pPar.g = 9.8 [m/(s^2)]
% ______
% p:
% Mass    -> d.pPar.m
% Moment of inertia -> d.pPar.Ixx, d.pPar.Iyy, d.pPar.Izz
% Rotation velocities -> d.pPar.W (MUST BE SET!)
% _________
% Pendulum:
% Mass -> p.pPar.m
% Pivot rod length -> p.pPar.r
% Moment of inertia -> p.pPar.I

% Simulation time tags:
% dt = toc(p.pPar.t) - p.pPar.ti; % Current simulation time
% p.pPar.ti = dt; % Updating flag time
dt = p.pPar.Ts;

% State variables:
q1 = p.pPos.Q(1); % alpha
q2 = p.pPos.Q(2); % beta
q3 = p.pPos.Q(3); % phi
q4 = p.pPos.Q(4); % theta
dq1 = p.pPos.dQ(1);  % dalpha 
dq2 = p.pPos.dQ(2);  % dbeta
dq3 = p.pPos.dQ(3);  % dphi
dq4 = p.pPos.dQ(4);  % dtheta
dq5 = p.pPos.dQ(5);  % dpsi
dq6 = p.pPos.dQ(6);  % dx
dq7 = p.pPos.dQ(7);  % dy
dq8 = p.pPos.dQ(8);  % dz
dq = [dq1, dq2, dq3, dq4, dq5, dq6, dq7, dq8];

%Constants:
g  = p.pPar.g; 
m  = p.pPar.m;
mc = p.pPar.mc;
r  = p.pPar.r;
Ix = p.pPar.Ix; 
Iy = p.pPar.Iy;
Iz = p.pPar.Iz;
I  = p.pPar.I;

% Rotation matrix:
RotX = [1 0 0; 0 cos(p.pPos.Q(3)) -sin(p.pPos.Q(3)); 0 sin(p.pPos.Q(3)) cos(p.pPos.Q(3))];
RotY = [cos(p.pPos.Q(4)) 0 sin(p.pPos.Q(4)); 0 1 0; -sin(p.pPos.Q(4)) 0 cos(p.pPos.Q(4))];
RotZ = [cos(p.pPos.Q(5)) -sin(p.pPos.Q(5)) 0; sin(p.pPos.Q(5)) cos(p.pPos.Q(5)) 0; 0 0 1];

R = RotZ*RotY*RotX;

% Simulate ArDrone dynamic model:
%         (1,2)             (3)              (4)                  
%      +----------+  W   +--------+  F   +----------+  T   +-------+
% U -> | Actuator |  ->  | Rotary |  ->  | Forces & |  ->  | Rigid |  -> Q
%      | Dynamics |      | Wing   |      | Torques  |      | Body  |
%      +----------+      +--------+      +----------+      +-------+
%
% (1) Receive input signal
%     pitch          | [-1,1] <==> [-15,15] degrees
%     roll           | [-1,1] <==> [-15,15] degrees
%     altitude rate  | [-1,1] <==> [-1,1] m/s
%     yaw rate       | [-1,1] <==> [-100,100] degrees/s

p.pSC.Qra = p.pSC.Qr;
p.pSC.dQra= p.pSC.dQr;

p.pSC.Qr(3)  =  p.pSC.Ud(1)*p.pPar.uSat(1);  % Phi
p.pSC.Qr(4)  = -p.pSC.Ud(2)*p.pPar.uSat(2);  % Theta
p.pSC.dQr(8) =  p.pSC.Ud(3)*p.pPar.uSat(3); % dZ
p.pSC.dQr(5) = -p.pSC.Ud(4)*p.pPar.uSat(4);  % dPsi

% Receive reference errors and compute forces to be applied to rigid body
% 2: Error -> Voltage
uphi   = p.pPar.kdp*(p.pSC.Qr(3)  -p.pPos.Q(3)  - p.pSC.Qra(3)  +p.pPos.Qa(3)) /p.pPar.Ts  + p.pPar.kpp*(p.pSC.Qr(3) -p.pPos.Q(3));
utheta = p.pPar.kdt*(p.pSC.Qr(4)  -p.pPos.Q(4)  - p.pSC.Qra(4)  +p.pPos.Qa(4)) /p.pPar.Ts  + p.pPar.kpt*(p.pSC.Qr(4) -p.pPos.Q(4)); 
udz    = p.pPar.kdz*(p.pSC.dQr(8) -p.pPos.dQ(8) - p.pSC.dQra(8) +p.pPos.dQa(8))/p.pPar.Ts  + p.pPar.kpz*(p.pSC.dQr(8)-p.pPos.dQ(8));
udpsi  = p.pPar.kds*(p.pSC.dQr(5) -p.pPos.dQ(5) - p.pSC.dQra(5) +p.pPos.dQa(5))/p.pPar.Ts  + p.pPar.kps*(p.pSC.dQr(5)-p.pPos.dQ(5));

p.pPar.V = p.pPar.Vo + (11.1-p.pPar.Vo)*[1 -1 1 1; 1 1 1 -1; -1 1 1 1; -1 -1 1 -1]*...
    [0.15*tanh(uphi); 0.15*tanh(utheta); 0.4*tanh(udz); 0.3*tanh(udpsi)];
    
% (2) Voltage -> Angular Velocities
% Motor dynamic model: 4 times faster than ArDrone dynamic model 
for ii = 1:4
p.pPar.W = 1/(p.pPar.Jm+p.pPar.Tsm*(p.pPar.Bm+p.pPar.Km*p.pPar.Kb/p.pPar.R))*...
    (p.pPar.Jm*p.pPar.W+p.pPar.Tsm*(p.pPar.Km/p.pPar.R*p.pPar.V-p.pPar.Ct*p.pPar.W.^2/p.pPar.r1));
end

% (3) Angular Velocities -> Forces and torques
% Deslocando valores passados
p.pPar.F  = p.pPar.Cf*p.pPar.W.^2;

% ArDrone coupling matrix 
At = R*[0 0 0 0; 0 0 0 0; 1 1 1 1];
Ar = [ p.pPar.k1  p.pPar.k1 -p.pPar.k1  -p.pPar.k1; ...
      -p.pPar.k1  p.pPar.k1  p.pPar.k1  -p.pPar.k1; ...
       p.pPar.k2 -p.pPar.k2  p.pPar.k2  -p.pPar.k2];
% torques in alpha and beta must be the contrary of torques in phi and theta
Ap = -Ar(1:2,:);  
A = [Ap;Ar;At];
% ft = R*At*p.pPar.F;
% T = Ar*p.pPar.F;
% ft = [-T(1:2,1); T; ft];
ft = A*p.pPar.F;
% ft= 0;
% Euler-Lagrange model:
%% Dynamic model:
% Inertia matrix
M = [...                                                                                            
  (I*sin(q1)^2 - I + I*sin(q2)^2 - 2*mc*r^2*cos(q1)^2 + mc*r^2*cos(q1)^2*sin(q1)^2 + mc*r^2*cos(q1)^2*sin(q2)^2)/(sin(q1)^2 + sin(q2)^2 - 1),                    0,           0,                                  0,                                                              0, mc*r*cos(q1),            0, -(2^(1/2)*mc*r*cos(q1))/(cos(2*q1) + cos(2*q2))^(1/2); ...
                                                                                                                                           0, I + mc*r^2*cos(q2)^2,           0,                                  0,                                                              0,            0, mc*r*cos(q2),                                                     0; ...
                                                                                                                                           0,                    0,          Ix,                                  0,                                                    -Ix*sin(q4),            0,            0,                                                     0; ...
                                                                                                                                           0,                    0,           0,   Iy - Iy*sin(q3)^2 + Iz*sin(q3)^2,                             -cos(q3)*cos(q4)*sin(q3)*(Iy + Iz),            0,            0,                                                     0; ...
                                                                                                                                           0,                    0, -Ix*sin(q4), -cos(q3)*cos(q4)*sin(q3)*(Iy + Iz), Iz*cos(q3)^2*cos(q4)^2 + Iy*cos(q4)^2*sin(q3)^2 + Ix*sin(q4)^2,            0,            0,                                                     0; ...
                                                                                                                                mc*r*cos(q1),                    0,           0,                                  0,                                                              0,       m + mc,            0,                                                     0; ...
                                                                                                                                           0,         mc*r*cos(q2),           0,                                  0,                                                              0,            0,       m + mc,                                                     0; ...
                                                                                       -(2^(1/2)*mc*r*cos(q1))/(cos(2*q1) + cos(2*q2))^(1/2),                    0,           0,                                  0,                                                              0,            0,            0,                                                m + mc];
% Coriolis matrix
C = [...
  (mc*r^2*cos(q1)*(2*dq1*cos(q1)^2*sin(q1) + dq1*cos(q2)^2*sin(q1) - dq1*cos(q1)^4*sin(q1) - dq1*cos(q2)^4*sin(q1) - 2*dq1*cos(q1)^2*cos(q2)^2*sin(q1) + dq2*cos(q1)*cos(q2)*sin(q2)))/(sin(q1)^2 + sin(q2)^2 - 1)^2, (dq1*mc*r^2*cos(q1)^2*cos(q2)*sin(q2))/(sin(q1)^2 + sin(q2)^2 - 1)^2 - (2^(1/2)*dq8*mc*r*sin(2*q2)*cos(q1))/(2*(cos(2*q1) + cos(2*q2))^(3/2)),                                                                                                            0,                                                                                                                                                                        0,                                                                                                                                                                                0, 0, 0, -(2^(1/2)*dq2*mc*r*sin(2*q2)*cos(q1))/(2*(cos(2*q1) + cos(2*q2))^(3/2)); ...
                                                                       (2^(1/2)*dq8*mc*r*sin(2*q2)*cos(q1))/(2*(cos(2*q1) + cos(2*q2))^(3/2)) - (dq1*mc*r^2*cos(q1)^2*cos(q2)*sin(q2))/(sin(q1)^2 + sin(q2)^2 - 1)^2,                                                                                                                     -(dq2*mc*r^2*sin(2*q2))/2,                                                                                                            0,                                                                                                                                                                        0,                                                                                                                                                                                0, 0, 0,  (2^(1/2)*dq1*mc*r*sin(2*q2)*cos(q1))/(2*(cos(2*q1) + cos(2*q2))^(3/2)); ...
                                                                                                                                                                                                                   0,                                                                                                                                             0,                                                                                                            0,                                                                                         (dq5*cos(q4)*(Iy*cos(2*q3) - Ix + Iz*cos(2*q3)))/2 + dq4*sin(2*q3)*(Iy/2 - Iz/2),                                                                     (dq4*cos(q4)*(Iy*(2*cos(q3)^2 - 1) - Ix + Iz*(2*cos(q3)^2 - 1)))/2 - dq5*cos(q3)*cos(q4)^2*sin(q3)*(Iy - Iz), 0, 0,                                                                       0; ...
                                                                                                                                                                                                                   0,                                                                                                                                             0,                           - (dq4*sin(2*q3)*(Iy - Iz))/2 - (dq5*cos(q4)*(Iy*cos(2*q3) - Ix + Iz*cos(2*q3)))/2,                                                                                                                                             -(dq3*sin(2*q3)*(Iy - Iz))/2, (cos(q4)*(Ix*dq3 - Iy*dq3 - Iz*dq3 + 2*Iy*dq3*sin(q3)^2 + 2*Iz*dq3*sin(q3)^2 - 2*Ix*dq5*sin(q4) + 2*Iz*dq5*sin(q4) + 2*Iy*dq5*sin(q3)^2*sin(q4) - 2*Iz*dq5*sin(q3)^2*sin(q4)))/2, 0, 0,                                                                       0; ...
                                                                                                                                                                                                                   0,                                                                                                                                             0, dq5*cos(q3)*cos(q4)^2*sin(q3)*(Iy - Iz) - (dq4*cos(q4)*(Ix + Iy*(2*cos(q3)^2 - 1) + Iz*(2*cos(q3)^2 - 1)))/2, dq5*cos(q4)*sin(q4)*(Ix - Iz - Iy*sin(q3)^2 + Iz*sin(q3)^2) - (dq3*cos(q4)*(Ix + Iy*(2*cos(q3)^2 - 1) + Iz*(2*cos(q3)^2 - 1)))/2 + dq4*cos(q3)*sin(q3)*sin(q4)*(Iy + Iz),                                                                            dq4*cos(q4)*sin(q4)*(Ix - Iz - Iy*sin(q3)^2 + Iz*sin(q3)^2) + dq3*cos(q3)*cos(q4)^2*sin(q3)*(Iy - Iz), 0, 0,                                                                       0; ...
                                                                                                                                                                                                   -dq1*mc*r*sin(q1),                                                                                                                                             0,                                                                                                            0,                                                                                                                                                                        0,                                                                                                                                                                                0, 0, 0,                                                                       0; ...
                                                                                                                                                                                                                   0,                                                                                                                             -dq2*mc*r*sin(q2),                                                                                                            0,                                                                                                                                                                        0,                                                                                                                                                                                0, 0, 0,                                                                       0; ...
                                                                         -(2*2^(1/2)*dq1*mc*r*sin(q1) - 2*2^(1/2)*dq1*mc*r*cos(2*q2)*sin(q1) + 2^(1/2)*dq2*mc*r*sin(2*q2)*cos(q1))/(2*(cos(2*q1) + cos(2*q2))^(3/2)),                                                                       -(2^(1/2)*dq1*mc*r*sin(2*q2)*cos(q1))/(2*(cos(2*q1) + cos(2*q2))^(3/2)),                                                                                                            0,                                                                                                                                                                        0,                                                                                                                                                                                0, 0, 0,                                                                       0    ];

% Gravity force matrix
G = [...
     -(2^(1/2)*g*mc*r*sin(2*q1))/(2*(cos(2*q1) + cos(2*q2))^(1/2)); ...
     -(2^(1/2)*g*mc*r*sin(2*q2))/(2*(cos(2*q1) + cos(2*q2))^(1/2)); ...
                                                                 0; ...
                                                                 0; ...
                                                                 0; ...
                                                                 0; ...
                                                                 0; ...
                                                         g*(m + mc)    ];
                                                     
%% Time discretization
% (5) Current system configuration by numerical integration:
% p.pPos.ddQ = M\(ft -G -C*dq')*dt + p.pPos.ddQ;
p.pPos.dQ = M\(ft -G -C*dq')*dt + p.pPos.dQ;
% p.pPos.dQ = (p.pPos.Q -p.pPos.Qa)/dt + p.pPos.dQ;
p.pPos.Q = p.pPos.dQ*dt + p.pPos.Q;

% Update previous pose
p.pPos.Qa  = p.pPos.Q;
p.pPos.dQa = p.pPos.dQ;
end
                                                  