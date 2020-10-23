function sDynamicModel(drone)

% Dynamic model from
% Brandão, A. S., M. Sarcinelli-Filho, and R. Carelli. 
% "High-level underactuated nonlinear control for rotorcraft machines." 
% Mechatronics (ICM), 2013 IEEE International Conference on. IEEE, 2013.
%
% ArDrone 2.0 Parameters
% Li, Qianying. "Grey-box system identification of a quadrotor unmanned 
% aerial vehicle." Master of Science Thesis Delft University of
% Technology (2014).
%
% Simulate ArDrone dynamic model
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

drone.pPar.Xra = drone.pPar.Xr;

drone.pPar.Xr(4)  =  drone.pSC.Ud(1)*drone.pPar.uSat(1);
drone.pPar.Xr(5)  = -drone.pSC.Ud(2)*drone.pPar.uSat(2);
drone.pPar.Xr(9)  =  drone.pSC.Ud(3)*drone.pPar.uSat(3);
drone.pPar.Xr(12) = -drone.pSC.Ud(4)*drone.pPar.uSat(4);

% Receive the reference errors and compute the forces to be applied to the
% rigid body
% 2: Error -> Voltage

uphi   = drone.pPar.kdp*(drone.pPar.Xr(4) -drone.pPos.X(4)  - drone.pPar.Xra(4) +drone.pPos.Xa(4) )/drone.pPar.Ts   + drone.pPar.kpp*(drone.pPar.Xr(4)-drone.pPos.X(4));
utheta = drone.pPar.kdt*(drone.pPar.Xr(5) -drone.pPos.X(5)  - drone.pPar.Xra(5) +drone.pPos.Xa(5) )/drone.pPar.Ts  + drone.pPar.kpt*(drone.pPar.Xr(5)-drone.pPos.X(5)); 
udz    = drone.pPar.kdz*(drone.pPar.Xr(9) -drone.pPos.X(9)  - drone.pPar.Xra(9) +drone.pPos.Xa(9) )/drone.pPar.Ts  + drone.pPar.kpz*(drone.pPar.Xr(9)-drone.pPos.X(9));
udpsi  = drone.pPar.kds*(drone.pPar.Xr(12)-drone.pPos.X(12) - drone.pPar.Xra(12)+drone.pPos.Xa(12))/drone.pPar.Ts  + drone.pPar.kps*(drone.pPar.Xr(12)-drone.pPos.X(12));

drone.pPar.V = drone.pPar.Vo + (11.1-drone.pPar.Vo)*[1 -1 1 1; 1 1 1 -1; -1 1 1 1; -1 -1 1 -1]*...
    [0.15*tanh(uphi); 0.15*tanh(utheta); 0.4*tanh(udz); 0.3*tanh(udpsi)];
    
% Saturation considering the limits of the energy source (battery)
% drone.pPar.V = (drone.pPar.V>0).*drone.pPar.V;
% drone.pPar.V = (drone.pPar.V<=11.1).*drone.pPar.V + (drone.pPar.V>11.1).*11.1;
% disp(drone.pPar.V)

% 2: V -> W
% Motor dynamic model: 4 times faster than ArDrone dynamic model 
for ii = 1:4
drone.pPar.W = 1/(drone.pPar.Jm+drone.pPar.Tsm*(drone.pPar.Bm+drone.pPar.Km*drone.pPar.Kb/drone.pPar.R))*...
    (drone.pPar.Jm*drone.pPar.W+drone.pPar.Tsm*(drone.pPar.Km/drone.pPar.R*drone.pPar.V-drone.pPar.Ct*drone.pPar.W.^2/drone.pPar.r));
end

% 3: W -> F
% Deslocando valores passados
drone.pPar.F  = drone.pPar.Cf*drone.pPar.W.^2;

% Euler-Lagrange model
drone.pPos.Xa = drone.pPos.X;

Rx = [1 0 0; 0 cos(drone.pPos.X(4)) -sin(drone.pPos.X(4)); 0 sin(drone.pPos.X(4)) cos(drone.pPos.X(4))];
Ry = [cos(drone.pPos.X(5)) 0 sin(drone.pPos.X(5)); 0 1 0; -sin(drone.pPos.X(5)) 0 cos(drone.pPos.X(5))];
Rz = [cos(drone.pPos.X(6)) -sin(drone.pPos.X(6)) 0; sin(drone.pPos.X(6)) cos(drone.pPos.X(6)) 0; 0 0 1];

R = Rz*Ry*Rx;

% =========================================================================
% Translational inertial matrix
% Matriz de inércia translacional
Mt = drone.pPar.m*eye(3,3);

% Gravitational vecto
G = [0; 0; drone.pPar.m*drone.pPar.g];

% ArDrone force matrix 
At = [0 0 0 0; 0 0 0 0; 1 1 1 1];


% Disturbance vector
ft = R*At*drone.pPar.F - drone.pPar.D(1:3);

% Numerical integration for Cartesian velocities
drone.pPos.X(7:9) = Mt\(ft - G)*drone.pPar.Ts + drone.pPos.X(7:9);

% =========================================================================
% Rotational inertia matrix
Mr = [drone.pPar.Ixx, ...
    drone.pPar.Ixy*cos(drone.pPos.X(4)) - drone.pPar.Ixz*sin(drone.pPos.X(4)), ...
    -drone.pPar.Ixx*sin(drone.pPos.X(5)) + drone.pPar.Ixy*sin(drone.pPos.X(4))*cos(drone.pPos.X(5)) + drone.pPar.Ixz*cos(drone.pPos.X(4))*cos(drone.pPos.X(5));
    
    drone.pPar.Ixy*cos(drone.pPos.X(4)) - drone.pPar.Ixz*sin(drone.pPos.X(4)), ...
    drone.pPar.Iyy*cos(drone.pPos.X(4))^2 + drone.pPar.Izz*sin(drone.pPos.X(4))^2 - 2*drone.pPar.Iyz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4)),...
    drone.pPar.Iyy*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*cos(drone.pPos.X(5)) - drone.pPar.Izz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*cos(drone.pPos.X(5)) - drone.pPar.Ixy*cos(drone.pPos.X(4))*sin(drone.pPos.X(5)) + drone.pPar.Ixz*sin(drone.pPos.X(4))*sin(drone.pPos.X(5)) + drone.pPar.Iyz*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5)) - drone.pPar.Iyz*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5));
    
    -drone.pPar.Ixx*sin(drone.pPos.X(5)) + drone.pPar.Ixy*sin(drone.pPos.X(4))*cos(drone.pPos.X(5)) + drone.pPar.Ixz*cos(drone.pPos.X(4))*cos(drone.pPos.X(5)), ...
    drone.pPar.Iyy*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*cos(drone.pPos.X(5)) - drone.pPar.Izz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*cos(drone.pPos.X(5)) - drone.pPar.Ixy*cos(drone.pPos.X(4))*sin(drone.pPos.X(5)) + drone.pPar.Ixz*sin(drone.pPos.X(4))*sin(drone.pPos.X(5)) + drone.pPar.Iyz*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5)) - drone.pPar.Iyz*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5)),...
    drone.pPar.Ixx*sin(drone.pPos.X(5))^2 + drone.pPar.Iyy*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5))^2 + drone.pPar.Izz*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5))^2 - 2*drone.pPar.Ixy*sin(drone.pPos.X(4))*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)) - 2*drone.pPar.Ixz*cos(drone.pPos.X(4))*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)) + 2*drone.pPar.Iyz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*cos(drone.pPos.X(5))^2
    ];

% Rotational Coriolis matrix
Cr = [ 0, ...
    drone.pPos.X(11)*(drone.pPar.Iyy*sin(drone.pPos.X(4))*cos(drone.pPos.X(5)) - drone.pPar.Izz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4)) + drone.pPar.Iyz*cos(drone.pPos.X(4))^2 - drone.pPar.Iyz*sin(drone.pPos.X(4))^2) + drone.pPos.X(12)*(-drone.pPar.Ixx*cos(drone.pPos.X(5))/2 - drone.pPar.Iyy*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 + drone.pPar.Iyy*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 + drone.pPar.Izz*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 - drone.pPar.Izz*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 - drone.pPar.Ixy*sin(drone.pPos.X(4))*sin(drone.pPos.X(5)) - drone.pPar.Ixz*cos(drone.pPos.X(4))*sin(drone.pPos.X(5)) + 2*drone.pPar.Iyz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*cos(drone.pPos.X(5))),...
    drone.pPos.X(11)*(-drone.pPar.Ixx*cos(drone.pPos.X(5))/2 - drone.pPar.Iyy*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 + drone.pPar.Iyy*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 + drone.pPar.Izz*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 - drone.pPar.Izz*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 - drone.pPar.Ixy*sin(drone.pPos.X(4))*sin(drone.pPos.X(5)) - drone.pPar.Ixz*cos(drone.pPos.X(4))*sin(drone.pPos.X(5)) + 2*drone.pPar.Iyz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*cos(drone.pPos.X(5))) + drone.pPos.X(12)*(-drone.pPar.Iyy*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*cos(drone.pPos.X(5))^2 + drone.pPar.Izz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*cos(drone.pPos.X(5))^2 + drone.pPar.Ixy*cos(drone.pPos.X(4))*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)) - drone.pPar.Ixz*sin(drone.pPos.X(4))*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)) - drone.pPar.Iyz*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5))^2 + drone.pPar.Iyz*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5))^2);
    
    drone.pPos.X(10)*(-drone.pPar.Ixy*sin(drone.pPos.X(4)) - drone.pPar.Ixz*cos(drone.pPos.X(4))) + drone.pPos.X(11)*(-drone.pPar.Iyy*sin(drone.pPos.X(4))*cos(drone.pPos.X(4)) + drone.pPar.Izz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4)) - drone.pPar.Iyz*cos(drone.pPos.X(4))^2 + drone.pPar.Iyz*sin(drone.pPos.X(4))^2) + drone.pPos.X(12)*(drone.pPar.Ixx*cos(drone.pPos.X(5))/2 + drone.pPar.Iyy*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 - drone.pPar.Iyy*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 - drone.pPar.Izz*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 + drone.pPar.Izz*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 + drone.pPar.Ixy*sin(drone.pPos.X(4))*sin(drone.pPos.X(5)) + drone.pPar.Ixz*cos(drone.pPos.X(4))*sin(drone.pPos.X(5)) - 2*drone.pPar.Iyz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*cos(drone.pPos.X(5))),...
    drone.pPos.X(10)*(-drone.pPar.Iyy*sin(drone.pPos.X(4))*cos(drone.pPos.X(4)) + drone.pPar.Izz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4)) - drone.pPar.Iyz*cos(drone.pPos.X(4))^2 + drone.pPar.Iyz*sin(drone.pPos.X(4))^2),...
    drone.pPos.X(10)*(drone.pPar.Ixx*cos(drone.pPos.X(5))/2 + drone.pPar.Iyy*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 - drone.pPar.Iyy*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 - drone.pPar.Izz*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 + drone.pPar.Izz*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 + drone.pPar.Ixy*sin(drone.pPos.X(4))*sin(drone.pPos.X(5)) + drone.pPar.Ixz*cos(drone.pPos.X(4))*sin(drone.pPos.X(5)) - 2*drone.pPar.Iyz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*cos(drone.pPos.X(5))) + drone.pPos.X(12)*(-drone.pPar.Ixx*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)) + drone.pPar.Iyy*sin(drone.pPos.X(4))^2*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)) + drone.pPar.Izz*cos(drone.pPos.X(4))^2*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)) + drone.pPar.Ixy*sin(drone.pPos.X(4))*cos(drone.pPos.X(5))^2 - drone.pPar.Ixy*sin(drone.pPos.X(4))*sin(drone.pPos.X(5))^2 + drone.pPar.Ixz*cos(drone.pPos.X(4))*cos(drone.pPos.X(5))^2 - drone.pPar.Ixz*cos(drone.pPos.X(4))*sin(drone.pPos.X(5))^2 + 2*drone.pPar.Iyz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)));
    
    drone.pPos.X(10)*(drone.pPar.Ixy*cos(drone.pPos.X(4))*cos(drone.pPos.X(5)) - drone.pPar.Ixz*sin(drone.pPos.X(4))*cos(drone.pPos.X(5))) + drone.pPos.X(11)*(-drone.pPar.Ixx*cos(drone.pPos.X(5))/2 + drone.pPar.Iyy*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 - drone.pPar.Iyy*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 - drone.pPar.Izz*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 + drone.pPar.Izz*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 - 2*drone.pPar.Iyz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*cos(drone.pPos.X(5))) + drone.pPos.X(12)*(drone.pPar.Iyy*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*cos(drone.pPos.X(5))^2 - drone.pPar.Izz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*cos(drone.pPos.X(5))^2 - drone.pPar.Ixy*cos(drone.pPos.X(4))*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)) + drone.pPar.Ixz*sin(drone.pPos.X(4))*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)) + drone.pPar.Iyz*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5))^2 - drone.pPar.Iyz*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5))^2),...
    drone.pPos.X(10)*(-drone.pPar.Ixx*cos(drone.pPos.X(5))/2 + drone.pPar.Iyy*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 - drone.pPar.Iyy*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 - drone.pPar.Izz*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 + drone.pPar.Izz*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5))/2 - 2*drone.pPar.Iyz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*cos(drone.pPos.X(5))) + drone.pPos.X(11)*(-drone.pPar.Iyy*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*sin(drone.pPos.X(5)) + drone.pPar.Izz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*sin(drone.pPos.X(5)) - drone.pPar.Ixy*cos(drone.pPos.X(4))*cos(drone.pPos.X(5)) + drone.pPar.Ixz*sin(drone.pPos.X(4))*cos(drone.pPos.X(5)) + drone.pPar.Iyz*sin(drone.pPos.X(4))^2*sin(drone.pPos.X(5)) - drone.pPar.Iyz*cos(drone.pPos.X(4))^2*sin(drone.pPos.X(5))) + drone.pPos.X(12)*(drone.pPar.Ixx*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)) - drone.pPar.Iyy*sin(drone.pPos.X(4))^2*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)) - drone.pPar.Izz*cos(drone.pPos.X(4))^2*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)) - drone.pPar.Ixy*sin(drone.pPos.X(4))*cos(drone.pPos.X(5))^2 + drone.pPar.Ixy*sin(drone.pPos.X(4))*sin(drone.pPos.X(5))^2 - drone.pPar.Ixz*cos(drone.pPos.X(4))*cos(drone.pPos.X(5))^2 + drone.pPar.Ixz*cos(drone.pPos.X(4))*sin(drone.pPos.X(5))^2 - 2*drone.pPar.Iyz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*sin(drone.pPos.X(5))*cos(drone.pPos.X(5))),...
    drone.pPos.X(10)*(drone.pPar.Iyy*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*cos(drone.pPos.X(5))^2 - drone.pPar.Izz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*cos(drone.pPos.X(5))^2 - drone.pPar.Ixy*cos(drone.pPos.X(4))*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)) + drone.pPar.Ixz*sin(drone.pPos.X(4))*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)) + drone.pPar.Iyz*cos(drone.pPos.X(4))^2*cos(drone.pPos.X(5))^2 - drone.pPar.Iyz*sin(drone.pPos.X(4))^2*cos(drone.pPos.X(5))^2) + drone.pPos.X(11)*(drone.pPar.Ixx*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)) - drone.pPar.Iyy*sin(drone.pPos.X(4))^2*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)) - drone.pPar.Izz*cos(drone.pPos.X(4))^2*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)) - drone.pPar.Ixy*sin(drone.pPos.X(4))*cos(drone.pPos.X(5))^2 + drone.pPar.Ixy*sin(drone.pPos.X(4))*sin(drone.pPos.X(5))^2 - drone.pPar.Ixz*cos(drone.pPos.X(4))*cos(drone.pPos.X(5))^2 + drone.pPar.Ixz*cos(drone.pPos.X(4))*sin(drone.pPos.X(5))^2 - 2*drone.pPar.Iyz*sin(drone.pPos.X(4))*cos(drone.pPos.X(4))*sin(drone.pPos.X(5))*cos(drone.pPos.X(5)))
    ];

% ArDrone
Ar = [drone.pPar.k1  drone.pPar.k1 -drone.pPar.k1  -drone.pPar.k1;
    -drone.pPar.k1  drone.pPar.k1  drone.pPar.k1  -drone.pPar.k1;
    drone.pPar.k2 -drone.pPar.k2  drone.pPar.k2  -drone.pPar.k2];

% Aerodynamic thrust 
T = Ar*drone.pPar.F - drone.pPar.Q;

%--------------------------------------------
% Numerical integration of rotational movement
drone.pPos.X(10:12) = Mr\(T - Cr*drone.pPos.X(10:12))*drone.pPar.Ts + drone.pPos.X(10:12);

% ArDrone pose - Numerical integration
for ii = 1:6
    drone.pPos.X(ii) = drone.pPos.X(ii+6)*drone.pPar.Ts + drone.pPos.X(ii);
    if ii > 3
        if drone.pPos.X(ii) > pi
            drone.pPos.X(ii) = -2*pi + drone.pPos.X(ii);
        end
        if drone.pPos.X(ii) < -pi
            drone.pPos.X(ii) = 2*pi + drone.pPos.X(ii);
        end
    end
end