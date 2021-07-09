function sDynamicModel(Catamara)

% Dynamic model from
% Brand�o, A. S., M. Sarcinelli-Filho, and R. Carelli. 
% "High-level underactuated nonlinear control for rotorcraft machines." 
% Mechatronics (ICM), 2013 IEEE International Conference on. IEEE, 2013.
%
% ArCatamara 2.0 Parameters
% Li, Qianying. "Grey-box system identification of a quadrotor unmanned 
% aerial vehicle." Master of Science Thesis Delft University of
% Technology (2014).
%
% Simulate ArCatamara dynamic model
%
%      +----------+  W   +--------+  F   +----------+  T   +-------+
% U -> | Actuator |  ->  | Rotary |  ->  | Forces & |  ->  | Rigid |  -> X
%      | Dynamics |      | Wing   |      | Torques  |      | Body  |
%      +----------+      +--------+      +----------+      +-------+
%


% 1: Receive input signal
%     pitch          | [-1,1] <==> [-15,15] degrees
%     roll           | [-1,1] <==> [-15,15] degrees
%     altitude rate  | [-1,1] <==> [-1,1] m/s
%     yaw rate       | [-1,1] <==> [-100,100] degrees/s

Catamara.pPar.Xra = Catamara.pPar.Xr;

Catamara.pPar.Xr(4)  =  Catamara.pSC.Ud(1)*Catamara.pPar.uSat(1);
Catamara.pPar.Xr(5)  = -Catamara.pSC.Ud(2)*Catamara.pPar.uSat(2);
Catamara.pPar.Xr(9)  =  Catamara.pSC.Ud(3)*Catamara.pPar.uSat(3);
Catamara.pPar.Xr(12) = -Catamara.pSC.Ud(4)*Catamara.pPar.uSat(4);

% Receive the reference errors and compute the forces to be applied to the
% rigid body
% 2: Error -> Voltage

uphi   = Catamara.pPar.kdp*(Catamara.pPar.Xr(4) -Catamara.pPos.X(4)  - Catamara.pPar.Xra(4) +Catamara.pPos.Xa(4) )/Catamara.pPar.Ts   + Catamara.pPar.kpp*(Catamara.pPar.Xr(4)-Catamara.pPos.X(4));
utheta = Catamara.pPar.kdt*(Catamara.pPar.Xr(5) -Catamara.pPos.X(5)  - Catamara.pPar.Xra(5) +Catamara.pPos.Xa(5) )/Catamara.pPar.Ts  + Catamara.pPar.kpt*(Catamara.pPar.Xr(5)-Catamara.pPos.X(5)); 
udz    = Catamara.pPar.kdz*(Catamara.pPar.Xr(9) -Catamara.pPos.X(9)  - Catamara.pPar.Xra(9) +Catamara.pPos.Xa(9) )/Catamara.pPar.Ts  + Catamara.pPar.kpz*(Catamara.pPar.Xr(9)-Catamara.pPos.X(9));
udpsi  = Catamara.pPar.kds*(Catamara.pPar.Xr(12)-Catamara.pPos.X(12) - Catamara.pPar.Xra(12)+Catamara.pPos.Xa(12))/Catamara.pPar.Ts  + Catamara.pPar.kps*(Catamara.pPar.Xr(12)-Catamara.pPos.X(12));

Catamara.pPar.V = Catamara.pPar.Vo + (11.1-Catamara.pPar.Vo)*[1 -1 1 1; 1 1 1 -1; -1 1 1 1; -1 -1 1 -1]*...
    [0.15*tanh(uphi); 0.15*tanh(utheta); 0.4*tanh(udz); 0.3*tanh(udpsi)];
    
% Saturation considering the limits of the energy source (battery)
% Catamara.pPar.V = (Catamara.pPar.V>0).*Catamara.pPar.V;
% Catamara.pPar.V = (Catamara.pPar.V<=11.1).*Catamara.pPar.V + (Catamara.pPar.V>11.1).*11.1;
% disp(Catamara.pPar.V)

% 2: V -> W
% Motor dynamic model: 4 times faster than ArCatamara dynamic model 
for ii = 1:4
Catamara.pPar.W = 1/(Catamara.pPar.Jm+Catamara.pPar.Tsm*(Catamara.pPar.Bm+Catamara.pPar.Km*Catamara.pPar.Kb/Catamara.pPar.R))*...
    (Catamara.pPar.Jm*Catamara.pPar.W+Catamara.pPar.Tsm*(Catamara.pPar.Km/Catamara.pPar.R*Catamara.pPar.V-Catamara.pPar.Ct*Catamara.pPar.W.^2/Catamara.pPar.r));
end

% 3: W -> F
% Deslocando valores passados
Catamara.pPar.F  = Catamara.pPar.Cf*Catamara.pPar.W.^2;

% Euler-Lagrange model
Catamara.pPos.Xa = Catamara.pPos.X;

Rx = [1 0 0; 0 cos(Catamara.pPos.X(4)) -sin(Catamara.pPos.X(4)); 0 sin(Catamara.pPos.X(4)) cos(Catamara.pPos.X(4))];
Ry = [cos(Catamara.pPos.X(5)) 0 sin(Catamara.pPos.X(5)); 0 1 0; -sin(Catamara.pPos.X(5)) 0 cos(Catamara.pPos.X(5))];
Rz = [cos(Catamara.pPos.X(6)) -sin(Catamara.pPos.X(6)) 0; sin(Catamara.pPos.X(6)) cos(Catamara.pPos.X(6)) 0; 0 0 1];

R = Rz*Ry*Rx;

% =========================================================================
% Translational inertial matrix
% Matriz de in�rcia translacional
Mt = Catamara.pPar.m*eye(3,3);

% Gravitational vector
G = [0; 0; Catamara.pPar.m*Catamara.pPar.g];

% ArCatamara force matrix 
At = [0 0 0 0; 0 0 0 0; 1 1 1 1];


% Disturbance vector
ft = R*At*Catamara.pPar.F - Catamara.pPar.D(1:3);

% Numerical integration for Cartesian velocities
Catamara.pPos.X(7:9) = Mt\(ft - G)*Catamara.pPar.Ts + Catamara.pPos.X(7:9);

% =========================================================================
% Rotational inertia matrix
Mr = [Catamara.pPar.Ixx, ...
    Catamara.pPar.Ixy*cos(Catamara.pPos.X(4)) - Catamara.pPar.Ixz*sin(Catamara.pPos.X(4)), ...
    -Catamara.pPar.Ixx*sin(Catamara.pPos.X(5)) + Catamara.pPar.Ixy*sin(Catamara.pPos.X(4))*cos(Catamara.pPos.X(5)) + Catamara.pPar.Ixz*cos(Catamara.pPos.X(4))*cos(Catamara.pPos.X(5));
    
    Catamara.pPar.Ixy*cos(Catamara.pPos.X(4)) - Catamara.pPar.Ixz*sin(Catamara.pPos.X(4)), ...
    Catamara.pPar.Iyy*cos(Catamara.pPos.X(4))^2 + Catamara.pPar.Izz*sin(Catamara.pPos.X(4))^2 - 2*Catamara.pPar.Iyz*sin(Catamara.pPos.X(4))*cos(Catamara.pPos.X(4)),...
    Catamara.pPar.Iyy*sin(Catamara.pPos.X(4))*cos(Catamara.pPos.X(4))*cos(Catamara.pPos.X(5)) - Catamara.pPar.Izz*sin(Catamara.pPos.X(4))*cos(Catamara.pPos.X(4))*cos(Catamara.pPos.X(5)) - Catamara.pPar.Ixy*cos(Catamara.pPos.X(4))*sin(Catamara.pPos.X(5)) + Catamara.pPar.Ixz*sin(Catamara.pPos.X(4))*sin(Catamara.pPos.X(5)) + Catamara.pPar.Iyz*cos(Catamara.pPos.X(4))^2*cos(Catamara.pPos.X(5)) - Catamara.pPar.Iyz*sin(Catamara.pPos.X(4))^2*cos(Catamara.pPos.X(5));
    
    -Catamara.pPar.Ixx*sin(Catamara.pPos.X(5)) + Catamara.pPar.Ixy*sin(Catamara.pPos.X(4))*cos(Catamara.pPos.X(5)) + Catamara.pPar.Ixz*cos(Catamara.pPos.X(4))*cos(Catamara.pPos.X(5)), ...
    Catamara.pPar.Iyy*sin(Catamara.pPos.X(4))*cos(Catamara.pPos.X(4))*cos(Catamara.pPos.X(5)) - Catamara.pPar.Izz*sin(Catamara.pPos.X(4))*cos(Catamara.pPos.X(4))*cos(Catamara.pPos.X(5)) - Catamara.pPar.Ixy*cos(Catamara.pPos.X(4))*sin(Catamara.pPos.X(5)) + Catamara.pPar.Ixz*sin(Catamara.pPos.X(4))*sin(Catamara.pPos.X(5)) + Catamara.pPar.Iyz*cos(Catamara.pPos.X(4))^2*cos(Catamara.pPos.X(5)) - Catamara.pPar.Iyz*sin(Catamara.pPos.X(4))^2*cos(Catamara.pPos.X(5)),...
    Catamara.pPar.Ixx*sin(Catamara.pPos.X(5))^2 + Catamara.pPar.Iyy*sin(Catamara.pPos.X(4))^2*cos(Catamara.pPos.X(5))^2 + Catamara.pPar.Izz*cos(Catamara.pPos.X(4))^2*cos(Catamara.pPos.X(5))^2 - 2*Catamara.pPar.Ixy*sin(Catamara.pPos.X(4))*sin(Catamara.pPos.X(5))*cos(Catamara.pPos.X(5)) - 2*Catamara.pPar.Ixz*cos(Catamara.pPos.X(4))*sin(Catamara.pPos.X(5))*cos(Catamara.pPos.X(5)) + 2*Catamara.pPar.Iyz*sin(Catamara.pPos.X(4))*cos(Catamara.pPos.X(4))*cos(Catamara.pPos.X(5))^2
    ];

% Rotational Coriolis matrix
Cr = [ 0, ...
    Catamara.pPos.X(11)*(Catamara.pPar.Iyy*sin(Catamara.pPos.X(4))*cos(Catamara.pPos.X(5)) - Catamara.pPar.Izz*sin(Catamara.pPos.X(4))*cos(Catamara.pPos.X(4)) + Catamara.pPar.Iyz*cos(Catamara.pPos.X(4))^2 - Catamara.pPar.Iyz*sin(Catamara.pPos.X(4))^2) + Catamara.pPos.X(12)*(-Catamara.pPar.Ixx*cos(Catamara.pPos.X(5))/2 - Catamara.pPar.Iyy*cos(Catamara.pPos.X(4))^2*cos(Catamara.pPos.X(5))/2 + Catamara.pPar.Iyy*sin(Catamara.pPos.X(4))^2*cos(Catamara.pPos.X(5))/2 + Catamara.pPar.Izz*cos(Catamara.pPos.X(4))^2*cos(Catamara.pPos.X(5))/2 - Catamara.pPar.Izz*sin(Catamara.pPos.X(4))^2*cos(Catamara.pPos.X(5))/2 - Catamara.pPar.Ixy*sin(Catamara.pPos.X(4))*sin(Catamara.pPos.X(5)) - Catamara.pPar.Ixz*cos(Catamara.pPos.X(4))*sin(Catamara.pPos.X(5)) + 2*Catamara.pPar.Iyz*sin(Catamara.pPos.X(4))*cos(Catamara.pPos.X(4))*cos(Catamara.pPos.X(5))),...
    Catamara.pPos.X(11)*(-Catamara.pPar.Ixx*cos(Catamara.pPos.X(5))/2 - Catamara.pPar.Iyy*cos(Catamara.pPos.X(4))^2*cos(Catamara.pPos.X(5))/2 + Catamara.pPar.Iyy*sin(Catamara.pPos.X(4))^2*cos(Catamara.pPos.X(5))/2 + Catamara.pPar.Izz*cos(Catamara.pPos.X(4))^2*cos(Catamara.pPos.X(5))/2 - Catamara.pPar.Izz*sin(Catamara.pPos.X(4))^2*cos(Catamara.pPos.X(5))/2 - Catamara.pPar.Ixy*sin(Catamara.pPos.X(4))*sin(Catamara.pPos.X(5)) - Catamara.pPar.Ixz*cos(Catamara.pPos.X(4))*sin(Catamara.pPos.X(5)) + 2*Catamara.pPar.Iyz*sin(Catamara.pPos.X(4))*cos(Catamara.pPos.X(4))*cos(Catamara.pPos.X(5))) + Catamara.pPos.X(12)*(-Catamara.pPar.Iyy*sin(Catamara.pPos.X(4))*cos(Catamara.pPos.X(4))*cos(Catamara.pPos.X(5))^2 + Catamara.pPar.Izz*sin(Catamara.pPos.X(4))*cos(Catamara.pPos.X(4))*cos(Catamara.pPos.X(5))^2 + Catamara.pPar.Ixy*cos(Catamara.pPos.X(4))*sin(Catamara.pPos.X(5))*cos(Catamara.pPos.X(5)) - Catamara.pPar.Ixz*sin(Catamara.pPos.X(4))*sin(Catamara.pPos.X(5))*cos(Catamara.pPos.X(5)) - Catamara.pPar.Iyz*cos(Catamara.pPos.X(4))^2*cos(Catamara.pPos.X(5))^2 + Catamara.pPar.Iyz*sin(Catamara.pPos.X(4))^2*cos(Catamara.pPos.X(5))^2);
    
    Catamara.pPos.X(10)*(-Catamara.pPar.Ixy*sin(Catamara.pPos.X(4)) - Catamara.pPar.Ixz*cos(Catamara.pPos.X(4))) + Catamara.pPos.X(11)*(-Catamara.pPar.Iyy*sin(Catamara.pPos.X(4))*cos(Catamara.pPos.X(4)) + Catamara.pPar.Izz*sin(Catamara.pPos.X(4))*cos(Catamara.pPos.X(4)) - Catamara.pPar.Iyz*cos(Catamara.pPos.X(4))^2 + Catamara.pPar.Iyz*sin(Catamara.pPos.X(4))^2) + Catamara.pPos.X(12)*(Catamara.pPar.Ixx*cos(Catamara.pPos.X(5))/2 + Catamara.pPar.Iyy*cos(Catamara.pPos.X(4))^2*cos(Catamara.pPos.X(5))/2 - Catamara.pPar.Iyy*sin(Catamara.pPos.X(4))^2*cos(Catamara.pPos.X(5))/2 - Catamara.pPar.Izz*cos(Catamara.pPos.X(4))^2*cos(Catamara.pPos.X(5))/2 + Catamara.pPar.Izz*sin(Catamara.pPos.X(4))^2*cos(Catamara.pPos.X(5))/2 + Catamara.pPar.Ixy*sin(Catamara.pPos.X(4))*sin(Catamara.pPos.X(5)) + Catamara.pPar.Ixz*cos(Catamara.pPos.X(4))*sin(Catamara.pPos.X(5)) - 2*Catamara.pPar.Iyz*sin(Catamara.pPos.X(4))*cos(Catamara.pPos.X(4))*cos(Catamara.pPos.X(5))),...
    Catamara.pPos.X(10)*(-Catamara.pPar.Iyy*sin(Catamara.pPos.X(4))*cos(Catamara.pPos.X(4)) + Catamara.pPar.Izz*sin(Catamara.pPos.X(4))*cos(Catamara.pPos.X(4)) - Catamara.pPar.Iyz*cos(Catamara.pPos.X(4))^2 + Catamara.pPar.Iyz*sin(Catamara.pPos.X(4))^2),...
    Catamara.pPos.X(10)*(Catamara.pPar.Ixx*cos(Catamara.pPos.X(5))/2 + Catamara.pPar.Iyy*cos(Catamara.pPos.X(4))^2*cos(Catamara.pPos.X(5))/2 - Catamara.pPar.Iyy*sin(Catamara.pPos.X(4))^2*cos(Catamara.pPos.X(5))/2 - Catamara.pPar.Izz*cos(Catamara.pPos.X(4))^2*cos(Catamara.pPos.X(5))/2 + Catamara.pPar.Izz*sin(Catamara.pPos.X(4))^2*cos(Catamara.pPos.X(5))/2 + Catamara.pPar.Ixy*sin(Catamara.pPos.X(4))*sin(Catamara.pPos.X(5)) + Catamara.pPar.Ixz*cos(Catamara.pPos.X(4))*sin(Catamara.pPos.X(5)) - 2*Catamara.pPar.Iyz*sin(Catamara.pPos.X(4))*cos(Catamara.pPos.X(4))*cos(Catamara.pPos.X(5))) + Catamara.pPos.X(12)*(-Catamara.pPar.Ixx*sin(Catamara.pPos.X(5))*cos(Catamara.pPos.X(5)) + Catamara.pPar.Iyy*sin(Catamara.pPos.X(4))^2*sin(Catamara.pPos.X(5))*cos(Catamara.pPos.X(5)) + Catamara.pPar.Izz*cos(Catamara.pPos.X(4))^2*sin(Catamara.pPos.X(5))*cos(Catamara.pPos.X(5)) + Catamara.pPar.Ixy*sin(Catamara.pPos.X(4))*cos(Catamara.pPos.X(5))^2 - Catamara.pPar.Ixy*sin(Catamara.pPos.X(4))*sin(Catamara.pPos.X(5))^2 + Catamara.pPar.Ixz*cos(Catamara.pPos.X(4))*cos(Catamara.pPos.X(5))^2 - Catamara.pPar.Ixz*cos(Catamara.pPos.X(4))*sin(Catamara.pPos.X(5))^2 + 2*Catamara.pPar.Iyz*sin(Catamara.pPos.X(4))*cos(Catamara.pPos.X(4))*sin(Catamara.pPos.X(5))*cos(Catamara.pPos.X(5)));
    
    Catamara.pPos.X(10)*(Catamara.pPar.Ixy*cos(Catamara.pPos.X(4))*cos(Catamara.pPos.X(5)) - Catamara.pPar.Ixz*sin(Catamara.pPos.X(4))*cos(Catamara.pPos.X(5))) + Catamara.pPos.X(11)*(-Catamara.pPar.Ixx*cos(Catamara.pPos.X(5))/2 + Catamara.pPar.Iyy*cos(Catamara.pPos.X(4))^2*cos(Catamara.pPos.X(5))/2 - Catamara.pPar.Iyy*sin(Catamara.pPos.X(4))^2*cos(Catamara.pPos.X(5))/2 - Catamara.pPar.Izz*cos(Catamara.pPos.X(4))^2*cos(Catamara.pPos.X(5))/2 + Catamara.pPar.Izz*sin(Catamara.pPos.X(4))^2*cos(Catamara.pPos.X(5))/2 - 2*Catamara.pPar.Iyz*sin(Catamara.pPos.X(4))*cos(Catamara.pPos.X(4))*cos(Catamara.pPos.X(5))) + Catamara.pPos.X(12)*(Catamara.pPar.Iyy*sin(Catamara.pPos.X(4))*cos(Catamara.pPos.X(4))*cos(Catamara.pPos.X(5))^2 - Catamara.pPar.Izz*sin(Catamara.pPos.X(4))*cos(Catamara.pPos.X(4))*cos(Catamara.pPos.X(5))^2 - Catamara.pPar.Ixy*cos(Catamara.pPos.X(4))*sin(Catamara.pPos.X(5))*cos(Catamara.pPos.X(5)) + Catamara.pPar.Ixz*sin(Catamara.pPos.X(4))*sin(Catamara.pPos.X(5))*cos(Catamara.pPos.X(5)) + Catamara.pPar.Iyz*cos(Catamara.pPos.X(4))^2*cos(Catamara.pPos.X(5))^2 - Catamara.pPar.Iyz*sin(Catamara.pPos.X(4))^2*cos(Catamara.pPos.X(5))^2),...
    Catamara.pPos.X(10)*(-Catamara.pPar.Ixx*cos(Catamara.pPos.X(5))/2 + Catamara.pPar.Iyy*cos(Catamara.pPos.X(4))^2*cos(Catamara.pPos.X(5))/2 - Catamara.pPar.Iyy*sin(Catamara.pPos.X(4))^2*cos(Catamara.pPos.X(5))/2 - Catamara.pPar.Izz*cos(Catamara.pPos.X(4))^2*cos(Catamara.pPos.X(5))/2 + Catamara.pPar.Izz*sin(Catamara.pPos.X(4))^2*cos(Catamara.pPos.X(5))/2 - 2*Catamara.pPar.Iyz*sin(Catamara.pPos.X(4))*cos(Catamara.pPos.X(4))*cos(Catamara.pPos.X(5))) + Catamara.pPos.X(11)*(-Catamara.pPar.Iyy*sin(Catamara.pPos.X(4))*cos(Catamara.pPos.X(4))*sin(Catamara.pPos.X(5)) + Catamara.pPar.Izz*sin(Catamara.pPos.X(4))*cos(Catamara.pPos.X(4))*sin(Catamara.pPos.X(5)) - Catamara.pPar.Ixy*cos(Catamara.pPos.X(4))*cos(Catamara.pPos.X(5)) + Catamara.pPar.Ixz*sin(Catamara.pPos.X(4))*cos(Catamara.pPos.X(5)) + Catamara.pPar.Iyz*sin(Catamara.pPos.X(4))^2*sin(Catamara.pPos.X(5)) - Catamara.pPar.Iyz*cos(Catamara.pPos.X(4))^2*sin(Catamara.pPos.X(5))) + Catamara.pPos.X(12)*(Catamara.pPar.Ixx*sin(Catamara.pPos.X(5))*cos(Catamara.pPos.X(5)) - Catamara.pPar.Iyy*sin(Catamara.pPos.X(4))^2*sin(Catamara.pPos.X(5))*cos(Catamara.pPos.X(5)) - Catamara.pPar.Izz*cos(Catamara.pPos.X(4))^2*sin(Catamara.pPos.X(5))*cos(Catamara.pPos.X(5)) - Catamara.pPar.Ixy*sin(Catamara.pPos.X(4))*cos(Catamara.pPos.X(5))^2 + Catamara.pPar.Ixy*sin(Catamara.pPos.X(4))*sin(Catamara.pPos.X(5))^2 - Catamara.pPar.Ixz*cos(Catamara.pPos.X(4))*cos(Catamara.pPos.X(5))^2 + Catamara.pPar.Ixz*cos(Catamara.pPos.X(4))*sin(Catamara.pPos.X(5))^2 - 2*Catamara.pPar.Iyz*sin(Catamara.pPos.X(4))*cos(Catamara.pPos.X(4))*sin(Catamara.pPos.X(5))*cos(Catamara.pPos.X(5))),...
    Catamara.pPos.X(10)*(Catamara.pPar.Iyy*sin(Catamara.pPos.X(4))*cos(Catamara.pPos.X(4))*cos(Catamara.pPos.X(5))^2 - Catamara.pPar.Izz*sin(Catamara.pPos.X(4))*cos(Catamara.pPos.X(4))*cos(Catamara.pPos.X(5))^2 - Catamara.pPar.Ixy*cos(Catamara.pPos.X(4))*sin(Catamara.pPos.X(5))*cos(Catamara.pPos.X(5)) + Catamara.pPar.Ixz*sin(Catamara.pPos.X(4))*sin(Catamara.pPos.X(5))*cos(Catamara.pPos.X(5)) + Catamara.pPar.Iyz*cos(Catamara.pPos.X(4))^2*cos(Catamara.pPos.X(5))^2 - Catamara.pPar.Iyz*sin(Catamara.pPos.X(4))^2*cos(Catamara.pPos.X(5))^2) + Catamara.pPos.X(11)*(Catamara.pPar.Ixx*sin(Catamara.pPos.X(5))*cos(Catamara.pPos.X(5)) - Catamara.pPar.Iyy*sin(Catamara.pPos.X(4))^2*sin(Catamara.pPos.X(5))*cos(Catamara.pPos.X(5)) - Catamara.pPar.Izz*cos(Catamara.pPos.X(4))^2*sin(Catamara.pPos.X(5))*cos(Catamara.pPos.X(5)) - Catamara.pPar.Ixy*sin(Catamara.pPos.X(4))*cos(Catamara.pPos.X(5))^2 + Catamara.pPar.Ixy*sin(Catamara.pPos.X(4))*sin(Catamara.pPos.X(5))^2 - Catamara.pPar.Ixz*cos(Catamara.pPos.X(4))*cos(Catamara.pPos.X(5))^2 + Catamara.pPar.Ixz*cos(Catamara.pPos.X(4))*sin(Catamara.pPos.X(5))^2 - 2*Catamara.pPar.Iyz*sin(Catamara.pPos.X(4))*cos(Catamara.pPos.X(4))*sin(Catamara.pPos.X(5))*cos(Catamara.pPos.X(5)))
    ];

% ArCatamara
Ar = [Catamara.pPar.k1  Catamara.pPar.k1 -Catamara.pPar.k1  -Catamara.pPar.k1;
    -Catamara.pPar.k1  Catamara.pPar.k1  Catamara.pPar.k1  -Catamara.pPar.k1;
    Catamara.pPar.k2 -Catamara.pPar.k2  Catamara.pPar.k2  -Catamara.pPar.k2];

% Aerodynamic thrust 
T = Ar*Catamara.pPar.F - Catamara.pPar.Q;

%--------------------------------------------
% Numerical integration of rotational movement
Catamara.pPos.X(10:12) = Mr\(T - Cr*Catamara.pPos.X(10:12))*Catamara.pPar.Ts + Catamara.pPos.X(10:12);

% ArCatamara pose - Numerical integration
for ii = 1:6
    Catamara.pPos.X(ii) = Catamara.pPos.X(ii+6)*Catamara.pPar.Ts + Catamara.pPos.X(ii);
    if ii > 3
        if Catamara.pPos.X(ii) > pi
            Catamara.pPos.X(ii) = -2*pi + Catamara.pPos.X(ii);
        end
        if Catamara.pPos.X(ii) < -pi
            Catamara.pPos.X(ii) = 2*pi + Catamara.pPos.X(ii);
        end
    end
end